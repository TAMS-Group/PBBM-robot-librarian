#!/usr/bin/env python3

import time

import numpy as np
import rospkg
import rospy
import torch
from cv_bridge import CvBridge
from librarian_craft import craft_utils as cu
from librarian_craft.infer import CraftInfer
from librarian_monitor.librarian_monitor import (draw_heatmaps,
                                                 draw_polys_on_source)
from librarian_resources.msg import BookDetections, BookPolys, ShelfImages
from rospy import Publisher, Subscriber, loginfo
from sensor_msgs.msg import Image


class DetectionNode:
    """ The detection node detectes book spine bounding boxes and text boxed.
        Also it publishes some monitoring information.
    """

    def __init__(self) -> None:
        pack_dir = rospkg.RosPack().get_path('librarian_vision_node')

        # Make sure the weights are present because their are ingored by git
        craft_weights = f"{pack_dir}/src/librarian_craft/craft_mlt_25k.pth"
        yolo_weights = f"{pack_dir}/src/librarian_segmentation/book_spine_model_v1.pt"

        # The CRAFT model for text detection (not recognition)
        self.craft = CraftInfer(cu, craft_weights, text_threshold=0.5)

        # The YOLO network for book spine detection / localization
        self.yolo = torch.hub.load(
            'ultralytics/yolov5', 'custom', path=yolo_weights)  # local model

        self.bridge = CvBridge()
        rospy.init_node('text_detection_node', anonymous=True)

        # Subscribe to the preprocessed shelf images
        Subscriber('/librarian/shelf_images', ShelfImages,
                   self.recieve_shelf_images, queue_size=1, buff_size=100_000)

        # Publish book detections containing polygons for
        # one book spine and the detected text
        self.pub_detections = Publisher(
            "/librarian/book/detections", BookDetections, queue_size=1)

        # Publishes an image showing the detected spines and texts
        self.pub_monitor_polys = Publisher(
            "/librarian/monitor/polys", Image, queue_size=1)

        # Published an image showing a heatmap from the detected text
        self.pub_monitor_heatmap = Publisher(
            "/librarian/monitor/heatmap", Image, queue_size=1)

        loginfo('text_detection_node ready')
        rospy.spin()

    def recieve_shelf_images(self, msg: ShelfImages):
        """ Processes the given msg. The characters will be detected 
            and polygons will be drawn around them. Additionally,
            heatmaps will be created and published.
        """
        text_polys = []
        spine_polys = []
        heatmaps = []
        source_images = []
        image_monitor_text_polys = []
        image_monitor_spine_polys = []

        num_detections = 0

        for shelfimg in msg.images:

            # read image data
            image = self.bridge.imgmsg_to_cv2(shelfimg.image, 'passthrough')

            # CRAFT forward pass
            _, craft_polys, heatmap = self.craft.infer(image)
            # YOLO forward pass
            result = self.yolo(image)
            p_r = result.pandas().xyxy[0]

            yolo_bbox = p_r.apply(lambda e: [[e.xmin, e.ymin], [e.xmax, e.ymin], [
                e.xmax, e.ymax], [e.xmin, e.ymax]], axis=1)

            yolo_bbox = yolo_bbox.to_numpy().tolist()
            num_detections += len(yolo_bbox)
            if len(yolo_bbox) == 0:
                continue 
                
            # prepare polygons for publish by unprojecting them
            projection = shelfimg.projection
            craft_polys = list(
                map(lambda x: self.unproject_polys(x, projection), craft_polys))
            yolo_bbox = list(
                map(lambda x: self.unproject_polys(x, projection), yolo_bbox))

            image_monitor_text_polys.extend(
                [[(*xy,) for xy in p] for p in craft_polys])
            image_monitor_spine_polys.extend(
                [[(*xy,) for xy in p] for p in yolo_bbox])

            text_polys.extend(np.array(craft_polys, dtype=int).tolist())
            spine_polys.extend(np.array(yolo_bbox, dtype=int).tolist())

            heatmap = np.array(heatmap)
            heatmaps.append(heatmap[:, :int(heatmap.shape[1]/2)])
            source_images.append(image)

        if num_detections == 0:
            rospy.logwarn("No book detections!")

        # create the text polys message
        text_polys = list(list(zip(*t)) for t in zip(*text_polys))
        text_polys_msg = self.book_polys_msg_from_polys(text_polys)

        # create the spine polys message
        spine_polys = list(list(zip(*t)) for t in zip(*spine_polys))
        if not spine_polys:
            return
        spine_polys_msg = self.book_polys_msg_from_polys(spine_polys)

        # create the combined version of both
        detections_msg = BookDetections(
            header=msg.header,
            source=msg.source,
            text_polys=text_polys_msg,
            spine_polys=spine_polys_msg
        )

        self.pub_detections.publish(detections_msg)

        # publish data to monitor
        monitor_polys = draw_polys_on_source(
            image_monitor_text_polys, image_monitor_spine_polys, msg)
        self.pub_monitor_polys.publish(monitor_polys)

        monitor_heatmap = draw_heatmaps(source_images, heatmaps)
        monitor_heatmap.header = msg.header
        self.pub_monitor_heatmap.publish(monitor_heatmap)

        # loginfo(f"Infer {time.time()-t:.4f}")

    def book_polys_msg_from_polys(self, polys):
        msg = BookPolys()
        if not polys:
            return msg
        msg.polys_x1 = polys[0][0]
        msg.polys_x2 = polys[1][0]
        msg.polys_x3 = polys[2][0]
        msg.polys_x4 = polys[3][0]
        msg.polys_y1 = polys[0][1]
        msg.polys_y2 = polys[1][1]
        msg.polys_y3 = polys[2][1]
        msg.polys_y4 = polys[3][1]
        return msg

    def unproject_polys(self, poly, m):
        """ Unprojecting the polygons is done using the projection matrix (m) and
            homographic transformation.
        """
        newpolys = []
        for p in poly:
            x = (m[0] * p[0] + m[1] * p[1] + m[2]) / \
                (m[6] * p[0] + m[7] * p[1] + 1)
            y = (m[3] * p[0] + m[4] * p[1] + m[5]) / \
                (m[6] * p[0] + m[7] * p[1] + 1)
            newpolys.append([x, y])
        return newpolys


if __name__ == '__main__':
    DetectionNode()
