#!/usr/bin/env python3

import rospy
import sys
from moveit_commander import roscpp_initialize, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from argparse import ArgumentParser
from rospkg import RosPack
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
from apriltag_ros.msg import AprilTagDetectionArray
from copy import deepcopy

scene = None
pub = None
tf_buffer = None

def parse_args():
    parser = ArgumentParser(prog='scene_setup.py',description='Script to set up moveit scene: a shelf and a book')
    parser.add_argument('-test',  action='store_true', help='Setup just book')
    parsed_args = parser.parse_args(sys.argv[1:])
    return parsed_args

def create_shelf(shelf_pose = None):
    '''
    Creates the shelf collision object in the moveit scene. The pose point for
    the shelf is its far, bottom right corner.

    :param PoseStamped shelf_pose: Pose where to spawn the shelf.
    '''
    global scene

    #probably to change orientations
    if shelf_pose == None:
        shelf_pose = PoseStamped()
        shelf_pose.header.frame_id = "shelf"

    rospack = RosPack()
    lib_res_path = rospack.get_path('librarian_resources')
    scene.add_mesh("shelf",shelf_pose, lib_res_path + '/meshes/shelf.stl')
    rospy.loginfo('Shelf was created.')

def create_book(book_pose = None, book_dimensions=(0.15, 0.025, 0.225), book_name = 'book'):
    '''
    Adds the book collision object to the moveit scene.

    :param PoseStamped book_pose: Pose of where to spawn the book. Set to None
        if the default position is desired.
    :param tuple book_dimensions: Tuple of three float64, which set the
        dimensions of the book object. (depth, width, height)
    :param String book_name: Name of the book object.
    '''
    global scene

    # If not book pose was given, add at a default position.
    if book_pose == None:
        book_pose = PoseStamped()
        book_pose.header.frame_id = 'shelf_0_0_0'
        book_pose.pose.position.x = 0.585
        book_pose.pose.position.y = 0.075
        book_pose.pose.position.z = 0.1125
        book_pose.pose.orientation.w = 0.7071068
        book_pose.pose.orientation.z = 0.7071068
    # If book pose was given relative to the camera frame, transform it.
    elif book_pose.header.frame_id == 'azure_kinect_rgb_camera_link':
        camera_base_transform = tf_buffer.lookup_transform('azure_kinect_rgb_camera_link', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
        book_pose = do_transform_pose(book_pose, camera_base_transform)

    scene.add_box(book_name, book_pose, size=book_dimensions)
    rospy.loginfo('Book was created.')

def delete_book(book_name = 'book'):
    '''
    Deletes the collision object book from the moveit scene. The object gets
    deleted whether it is attached or not.
    '''
    if 'book' in scene.get_objects():
        scene.remove_world_object(book_name)
        rospy.loginfo('Deleted the world collision object {}.'.format(book_name))
    elif 'book' in scene.get_attached_objects():
        scene.remove_attached_object(name=book_name)
        scene.remove_world_object(book_name)
        rospy.loginfo('Deleted the attached collision object {}.'.format(book_name))
    else:
        rospy.loginfo('No object with the name {} was found in the planning scene. No object was removed.'.format(book_name))


def test_spawn_book(detection_msg, books = None, book_list = [4, 1, 3], apriltag_id = 106):
    '''
    Spawns books next to each other to the left of an Apriltag. The Apriltag
    needs to lie on one of the shelfs ands the books need to stand in the
    provided list order from right to left next to the Apriltag. Useful for
    testing.

    :param AprilTagDetectionArray detection_msg: A message of all detected
        Apriltags.
    :param dict books: Dict of all spawnable books. Configuration as in the
        default one below. Set to None if default dict is desired.
    :param list book_list: List of int32 which describe which books are to be
        spawned. The int32 must align with keys in the books dict. The order
        in the list also describes the order of spawned books, with the first
        entry in the list spawning the most right book.
    :param int32 apriltag_id: The id of the Apriltag the books should be
        spawned next to. Needs to be defined in the config file.
    '''
    rospy.loginfo('Detection message received.')

    if books is None:
        # Dict of default spawnable books. size: d, w, h
        books = {1: {"name":"technik woertbuch", "size": (0.125, 0.018, 0.195)},
                2: {"name":"Die Suchfibel", "size": (0.175, 0.018, 0.245)},
                3: {"name":"Halbleiterbauelemente", "size": (0.126, 0.0265, 0.1865)},
                4: {"name":"Duden", "size": (0.11, 0.029, 0.167)},
                5: {"name":"TTL", "size": (0.149, 0.045, 0.207)}}

    # Filter poses of all detected Apriltags.
    markers=list(detection_msg.detections)
    poses=dict(zip([i.id[0] for i in markers], [i.pose.pose for i in markers]))

    try:
        create_shelf()
    except:
        rospy.loginfo('The shelf could not be created.')

    # Get the pose of the desired Apriltag.
    tag_pose = PoseStamped()
    tag_pose.header.frame_id = "azure_kinect_rgb_camera_link"
    try:
        tag_pose.pose = poses[apriltag_id].pose
    except KeyError as e:
        rospy.logerr('The Apriltag {} was not detected. Try again later.'.format(apriltag_id))
        return

    # Transform received tag pose to bottom left corner pose of the Apriltag in base_footprint.
    camera_base_transform = tf_buffer.lookup_transform('base_footprint', 'azure_kinect_rgb_camera_link', rospy.Time(0), rospy.Duration(1.0))
    tag_blcorner_pose = do_transform_pose(tag_pose, camera_base_transform)
    tag_blcorner_pose.pose.position.y += 0.0425 # Half of tag size
    tag_blcorner_pose.pose.position.z += -0.0425 # Half of tag size

    # Debugging publisher
    pub = rospy.Publisher('debugging_scene_setup', PoseStamped, queue_size = 1)
    rospy.sleep(1)
    # pub.publish(tag_blcorner_pose)

    last_blcorner_pose = tag_blcorner_pose

    # Spawn books next to each other left of the Apriltag.
    for k in book_list:
        try:
            book_name = books[k]["name"]
            book_dimensions = books[k]["size"]

            book_pose = deepcopy(last_blcorner_pose)
            book_pose.pose.position.x += book_dimensions[0]/2
            book_pose.pose.position.y += book_dimensions[1]/2
            book_pose.pose.position.z += book_dimensions[2]/2
            book_pose.pose.orientation.x = 0
            book_pose.pose.orientation.y = 0
            book_pose.pose.orientation.z = 0
            book_pose.pose.orientation.w = 1

            # pub.publish(book_pose)

            last_blcorner_pose.pose.position.y += book_dimensions[1] + 0.002 # Spwan book a little bit with a distance so they don't have a permanent collision

            scene.add_box(book_name, book_pose, size = (book_dimensions[0], book_dimensions[1]-0.003, book_dimensions[2]))
            rospy.loginfo('Book {} was created.'.format(book_name))

        except Exception as e:
            rospy.logerr('the book "{}" with marker {} was not created'.format(books[k]["name"], e))
    sys.exit(0)

def main_scenario():
    shelf_pose = PoseStamped()
    shelf_pose.header.frame_id = 'shelf'
    shelf_pose.pose.position.x += 0.005 # Moved shelf to the right
    shelf_pose.pose.position.y += 0.02 # Safety offset
    create_shelf(shelf_pose)
    create_book()

def test_scenario():
    rospy.Subscriber('tag_detections', AprilTagDetectionArray,  test_spawn_book)
    rospy.loginfo('Test setup complete.')
    rospy.spin()

def main():
    global scene, tf_buffer
    rospy.init_node('scene_setup')
    test = parse_args().test
    roscpp_initialize(sys.argv)
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer)
    scene = PlanningSceneInterface(synchronous=True)
    if test:
       test_scenario()
    else:
        main_scenario()


if __name__ == '__main__':
    main()

