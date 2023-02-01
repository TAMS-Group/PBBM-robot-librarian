#!/usr/bin/env python3

import rospy
from rospy import loginfo
from tf import TransformListener
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
import numpy as np
import cv2
from librarian_resources.msg import ShelfImage, ShelfImages, VisionControl
from cv_bridge import CvBridge
from librarian_monitor.librarian_monitor import shelf_images_to_monitor_msg


class PreProcNode:
    """ The preprocess node uses the shelf's corners to perform a
        perspective transform on each of the shelf's levels.
    """

    upper_shelf = ["1_0_1", "1_1_1", "1_1_0", "1_0_0"]
    lower_shelf = ["0_0_1", "0_1_1", "0_1_0", "0_0_0"]

    def __init__(self):
        self.padding = 20  # padding to the perspective transform
        self.apriltag_size = 0.17
        # whether or not to publish an image showing the results of the preprocessing
        self.monitor_preproc = True
        self.bridge = CvBridge()

        # Whether the vision pipeline is enabled or not
        self.enabled = True
        rospy.Subscriber("/librarian/control/vision",
                         VisionControl, self.receive_control)

        rospy.init_node('librarian_preproc_node')

        # Wait for the camera info to construct the pinhole camera model
        loginfo('Waiting for camera_info message')
        cam_msg = rospy.wait_for_message("/librarian/camera_info", CameraInfo)

        self.tfl = TransformListener()

        # Use the delayed images from 'librarian_shelf_from_image.py' for preprocessing
        self.image_sub = rospy.Subscriber(
            "/librarian/preproc_input/image_color", ImageMsg, lambda x: self.receive_img(x), queue_size=1, buff_size=100000)

        # The preprocessed version is one image for each of the shelf's levels.
        # A perspective transformation is applied to both images.
        self.shelf_image_pub = rospy.Publisher(
            "/librarian/shelf_images", ShelfImages, queue_size=1)

        # The monitoring image depicts both result images.
        self.image_pub = rospy.Publisher(
            "/librarian/image", ImageMsg, queue_size=1)

        # The pinhole camera model is used to get image coordinates
        # from the 3d shelf corner poses.
        self.pinhole = PinholeCameraModel()
        self.pinhole.fromCameraInfo(cam_msg)

        rospy.loginfo("Preprocessing Setup complete!")
        rospy.spin()

    def receive_control(self, msg: VisionControl):
        self.enabled = msg.enabled

    def get_tf(self, target_tf):
        return self.tfl.lookupTransform(
            self.src_tf, target_tf, rospy.Time(0))

    def get_shelf_ratio(self, positions):
        """ Returns the ratio from one of the shelf's levels given
            by the points top-left, top-right, bottom-left
        """
        p1, p2, p3 = positions
        d1 = np.linalg.norm(np.array(p1) - np.array(p2))
        d2 = np.linalg.norm(np.array(p1) - np.array(p3))
        return d2 / d1

    def receive_img(self, img):
        """ Performs the perspective transform on the input image and publishs
            the results for further processing in the image pipeline.
        """
        if not self.enabled:
            return


        # Use the messages frame as base frame for lookups
        # using self.get_tf
        self.src_tf = img.header.frame_id

        try:
            self.get_tf('shelf_1_0_1')
        except Exception as e:
            return rospy.loginfo('Waiting for book shelf frames')
        im = self.bridge.imgmsg_to_cv2(img, 'passthrough')

        # calculate the x,y image coordinates for all shelf corners for 'all' shelf levels
        upper_pos = list(map(lambda x: np.array(self.get_tf(
            "/shelf_" + x)[0]), PreProcNode.upper_shelf))

        # For the upper level, cut out the april tag as it only confuses further processing steps
        shelf_orientation = quaternion_matrix(self.get_tf("Shelf")[1])
        # The offset we apply to crop out the april tag
        offset = np.dot(shelf_orientation, [self.apriltag_size, 0, 0, 1])[:3]
        upper_pos[0] += offset
        upper_pos[3] += offset
        # Project world positions to screen coordinates
        upper_coords = list(
            map(lambda x: self.pinhole.project3dToPixel(x), upper_pos))

        # Compute the screen coordinated for the lower ölevel
        lower_pos = list(map(lambda x: self.get_tf(
            "/shelf_" + x)[0], PreProcNode.lower_shelf))
        # Project world positions to screen coordinates
        lower_coords = list(
            map(lambda x: self.pinhole.project3dToPixel(x), lower_pos))

        # calculate the new, transformed images (for each level)
        shelf_images = []
        monitor_imgs = []

        for pos, coords in [(upper_pos, upper_coords), (lower_pos, lower_coords)]:
            # calculate the shelves ratio
            ratio = self.get_shelf_ratio([pos[0], pos[1], pos[3]])

            # get the longest horizontal edge
            dist1 = np.linalg.norm(np.array(coords[0]) - np.array(coords[1]))
            dist2 = np.linalg.norm(np.array(coords[2]) - np.array(coords[3]))
            dist = max(dist1, dist2)

            # calculate the new width and height
            width = int(dist) + self.padding * 2
            height = int(dist * ratio) + self.padding * 2

            # Apply padding to not mistakenly crop out book spine information
            p = self.padding
            target_rect = np.array(
                [[p, p], [width - 1 - p, p],
                 [width - 1 - p, height - 1 - p], [p, height - 1 - p]
                 ], dtype=np.float32)
            source_rect = np.array(coords, dtype=np.float32)

            # compute the perspective transformation matrix
            pm = cv2.getPerspectiveTransform(source_rect, target_rect)

            # We also need the inverse to project from 'corrected image' to
            # the original 'distorted' image (which happens in 'detection_node.py')
            pm_inv = cv2.getPerspectiveTransform(target_rect, source_rect)

            # do the perspective transformation
            newim = cv2.warpPerspective(im, pm, (width, height))

            # add image to later publish a concatenated version for monitoring
            monitor_imgs.append(newim)

            # create the image message
            cimg = self.bridge.cv2_to_imgmsg(np.uint8(newim))
            cimg.header = img.header

            # create the shelf image message
            msg = ShelfImage()
            msg.image = cimg
            # flatten the projection matrix
            msg.projection = pm_inv.tolist(
            )[0] + pm_inv.tolist()[1] + pm_inv.tolist()[2]
            shelf_images.append(msg)

        # Publish the two corrected images
        msg = ShelfImages(header=img.header)
        msg.images = shelf_images
        msg.source = img
        self.shelf_image_pub.publish(msg)

        # Construct a monitoring image to verify the preprocessing works
        if self.monitor_preproc:
            monitor = shelf_images_to_monitor_msg(monitor_imgs)
            monitor.header =  msg.header
            self.image_pub.publish(monitor)


if __name__ == '__main__':
    try:
        PreProcNode()
    except Exception as e:
        print(e)
