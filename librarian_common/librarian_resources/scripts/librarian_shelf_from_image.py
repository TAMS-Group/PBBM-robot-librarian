#!/usr/bin/env python3

from copy import deepcopy

import cv2
import numpy as np
import PyKDL as kdl
import rospkg
import rospy
import tf2_geometry_msgs as tf2_gm
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
from librarian_resources.msg import VisionControl
from moveit_commander import PlanningSceneInterface
from rospy import Header, Publisher, Subscriber, Time, loginfo
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class ShelfFromImage:
    """ The 'shelf-from-image' node does two things: it republishes the raw camera image
        while computing the position of the shelf and its corners. Republishing happens
        because of two reasons: 
        1. The camera image size of the azure kinect can be up to 12MP. Te april tag detection
        necessary to construct the shelf does not need such high resolution images, so this
        node downscales the image and publishs it along with a manipulated camera info.
        2. The tf frames of the shelf are available with a short processing delay, so the
        camera image is being republished with a delay. This ensures that the following
        pipeline stages have the most up-to-date tf frames.
        Furthermore, when the pipeline is disabled (so the robot can move) the shelf frames
        are being republished by this node nonetheless. 
    """

    tag_size = 0.144 * 10 / 8  # Our april tag size
    lb_x = -tag_size / 2 + 0.016  # left bottom x
    lb_y = -(1.25 - 1.235) - 0.287 + tag_size / 2  # left bottom y

    # The corners of the shelf are defined by their offsets towards the april tag.
    shelf_corners = {
        # upper shelf
        "shelf_1_0_0": [lb_x, lb_y, 0],  # bottom left
        "shelf_1_1_0": [lb_x + 0.765, lb_y, 0],  # bottom right
        "shelf_1_1_1": [lb_x + 0.765, lb_y + 0.287, 0],  # top right
        "shelf_1_0_1": [lb_x, lb_y + 0.287, 0],  # top left
        # lower shelf
        "shelf_0_0_0": [lb_x, lb_y - 0.016 - 0.335, 0],  # bottom left
        "shelf_0_1_0": [lb_x + 0.765, lb_y - 0.016 - 0.335, 0],  # bottom right
        "shelf_0_1_1": [lb_x + 0.765, lb_y - 0.016, 0],  # top right
        "shelf_0_0_1": [lb_x, lb_y - 0.016, 0]  # top left
    }

    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.tfb = TransformBroadcaster()
        self.tag_size = ShelfFromImage.tag_size
        self.image_cache = {}  # cache original images by their stamp in seconds
        self.camera_info = None
        self.scene = PlanningSceneInterface(synchronous=True)
        resources = rospkg.RosPack().get_path("librarian_resources")
        self.shelf_mesh = f"{resources}/meshes/shelf.stl"
        self.vision_enabled = True  # Indicates whether the vision is enabled/disabled

        rospy.init_node('librarian_shelf_from_image', anonymous=True)

        # Subscribe to camera image to do downscaling and delayed republishing
        Subscriber('/librarian/input_image', Image,
                   lambda x: self.receive_image(x), queue_size=1)

        # Subscribe to camera info to republish manipulated version along with the
        # downscaled image for april tag detection
        Subscriber('/librarian/camera_info', CameraInfo,
                   lambda x: self.receive_camera_info(x), queue_size=1)

        # Subscribe to apriltag detections to compute the shelf's frames
        Subscriber('/tag_detections', AprilTagDetectionArray,
                   lambda x: self.receive_tag_detections(x), queue_size=1)

        # Subscribe to the vision pipeline enable/disable commands
        Subscriber('/librarian/control/vision', VisionControl,
                   self.receive_vision_control, queue_size=1)

        # Publishes smaller images to be processed by april tags
        self.pub_april_image = Publisher(
            '/librarian/apriltag_camera/image_color', Image, queue_size=1)

        # Publishes manipulated camera info to be used by april tags detection
        self.pub_caminfo = Publisher(
            '/librarian/apriltag_camera/camera_info', CameraInfo, queue_size=1)

        # Publishes raw input image but delayed so the tf frames are ready
        self.pub_librarian_image = Publisher(
            '/librarian/preproc_input/image_color', Image, queue_size=1)

        # The desired height for the downscaled image is set vis ros parameter
        self.target_height = int(rospy.get_param(
            "/librarian_shelf_from_image/apriltag_image_height", default="800"))
        loginfo(f"Target height for april tag detection: {self.target_height}")

        self.camera_info: CameraInfo

        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer)
        self.test_pub = Publisher('debug_apriltag', PoseStamped, queue_size=1)
        self.latest_transforms = []

        # Continuously republish the latest tf frames when the vision pipeline is disabled
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.vision_enabled and self.latest_transforms and self.camera_info:
                for t in self.latest_transforms:
                    t.header.stamp = Time.now()
                    self.tfb.sendTransform(t)
                self.update_shelf()
            rate.sleep()

    def receive_vision_control(self, msg: VisionControl):
        self.vision_enabled = msg.enabled

    def receive_camera_info(self, msg: CameraInfo):
        self.camera_info = msg

    def scale_camera_info(self, msg: CameraInfo, scale: float):
        """ This function scales the projection matrix of the original
            camera info by the scale determined by the target_height. """

        nmsg = CameraInfo(header=msg.header, D=msg.D, R=msg.R,
                          roi=msg.roi, distortion_model=msg.distortion_model)
        nmsg.width = int(msg.width * scale)
        nmsg.height = int(msg.height * scale)
        nmsg.K = (np.array(msg.K) * scale).tolist()
        nmsg.K[8] = 1
        nmsg.P = (np.array(msg.P) * scale).tolist()
        nmsg.P[10] = 1
        return nmsg

    def receive_image(self, msg: Image):
        """ This function downscales the image and republishes it
            to be processed by the april tag detection. """

        if self.camera_info == None or not self.vision_enabled:
            return

        # Downscale the source image
        image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')[:, :, :3]
        scale = self.target_height / image.shape[0]
        size = np.int32(np.array(image.shape[:2]) * scale)
        image_sm = cv2.resize(image, (size[1], size[0]))
        newmsg = self.bridge.cv2_to_imgmsg(image_sm, 'passthrough')
        newmsg.header = msg.header

        # Downscale the camera info
        cam_info = self.scale_camera_info(self.camera_info, scale)
        cam_info.header = msg.header
        self.pub_caminfo.publish(cam_info)

        # Publish small image after publishing camera info
        self.pub_april_image.publish(newmsg)

        # Cache original image
        self.image_cache[msg.header.stamp.to_sec()] = msg

    def receive_tag_detections(self, msg: AprilTagDetectionArray):
        # pick out the correct april tag
        detections = [d for d in msg.detections if d.id[0] == 10]
        if detections and self.vision_enabled:
            self.process_detection(msg.header, detections[0])
            self.update_shelf()

    def process_detection(self, header: Header, detection: AprilTagDetection):
        """ Uses one apriltag detection to compute the shelf frames
        """

        self.latest_transforms = []  # reset frame cache
        t = TransformStamped()
        t.header = deepcopy(header)

        # Make shelf relative to base_footprint rather than camera
        t.header.frame_id = 'base_footprint'
        camera_base_transform = self.tf_buffer.lookup_transform(
            'base_footprint', header.frame_id, Time(0))
        detection_pose = PoseStamped()
        detection_pose.header.frame_id = header.frame_id
        detection_pose.pose = detection.pose.pose.pose
        detection_pose_base = tf2_gm.do_transform_pose(
            detection_pose, camera_base_transform)

        # Publish the pose of the april tag itself
        t.child_frame_id = "shelf_tag"
        p = detection_pose_base.pose.position
        q = detection_pose_base.pose.orientation
        t.transform.translation = Point(x=p.x, y=p.y, z=p.z)
        t.transform.rotation = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
        self.latest_transforms.append(t)
        self.tfb.sendTransform(t)

        # Publish the shelf's own pose so we can place the shelf mesh
        o = detection_pose_base.pose.orientation
        p = detection_pose_base.pose.position
        T = kdl.Frame(kdl.Rotation.Quaternion(o.x, o.y, o.z, o.w),
                      kdl.Vector(p.x, p.y, p.z))
        tag_size = 0.144*10/8
        T_marker_to_shelf = kdl.Frame(kdl.Rotation.EulerZYX(np.pi/2, np.pi/2, 0)*kdl.Rotation.EulerZYX(np.pi/2, 0, 0),
                                      kdl.Vector((0.765+0.016*2-tag_size/2), -1.25+tag_size/2, -(0.36+0.016)))
        T_shelf = T*T_marker_to_shelf
        t = TransformStamped()
        t.header = deepcopy(header)
        t.child_frame_id = "shelf"
        t.header.frame_id = 'base_footprint'
        t.transform.translation.x = T_shelf.p[0]
        t.transform.translation.y = T_shelf.p[1]
        t.transform.translation.z = T_shelf.p[2]
        rotation = np.array(T_shelf.M.GetQuaternion())
        rotation = rotation / np.sqrt((rotation**2).sum())
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]
        self.latest_transforms.append(t)
        self.tfb.sendTransform(t)

        # Publish each corner of the shelf for later image processing
        for corner in ShelfFromImage.shelf_corners:
            offset = ShelfFromImage.shelf_corners[corner]
            t = TransformStamped()
            t.header = Header(stamp=header.stamp, seq=header.seq)
            t.header.frame_id = "shelf_tag"
            t.child_frame_id = corner
            t.transform.translation = Point(
                x=offset[0], y=offset[1], z=offset[2])
            # turn pi radians around x-axis
            t.transform.rotation = Quaternion(
                w=np.sqrt(2)/2, x=-np.sqrt(2)/2, y=0, z=0)
            self.latest_transforms.append(t)
            self.tfb.sendTransform(t)

        try:
            t = self.tf_buffer.lookup_transform('base_footprint', 'Shelf', Time(0))
            self.latest_transforms.append(t)
        except:
            pass

        # Publish the cached image now that we broadcasted all transforms
        stamp = header.stamp.to_sec()
        if stamp in self.image_cache.keys():
            img = self.image_cache[stamp]
            self.pub_librarian_image.publish(img)
            old_stamps = filter(lambda x: x <= stamp, self.image_cache.keys())
            for s in list(old_stamps):
                self.image_cache.pop(s)

    def update_shelf(self):
        """ Adds the shelf collision object to the moveit scene
            Center of the shelf is in the right far corner. If needed, can be edited and re-exported from scad
        """

        shelf_pose = PoseStamped()
        shelf_pose.header.frame_id = 'shelf'
        shelf_pose.pose.position.x += 0.005
        shelf_pose.pose.position.y += 0.015 + 0.01
        shelf_pose.pose.orientation.w = 1

        self.scene.add_mesh("shelf", shelf_pose, self.shelf_mesh)


if __name__ == '__main__':
    ShelfFromImage()
