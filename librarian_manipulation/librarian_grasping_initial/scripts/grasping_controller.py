#!/usr/bin/env python3

import rospy
import sys
from moveit_commander import RobotCommander, roscpp_initialize, MoveGroupCommander, PlanningSceneInterface
from argparse import ArgumentParser
from librarian_grasping_initial import GripperInterface, SHInterface
from librarian_resources.srv import ManipulationService
from std_msgs.msg import String
from moveit_commander.exception import MoveItCommanderException

demo = None
book_shelf = None
place_shelf = None
general_controllers = []
left_arm_controllers = []
right_arm_controllers = []

def parse_args():
    '''
    Parses the arguments when the file was launched to check, if the demo
    argument was given.

    :return list: List of parsed arguments.
    '''
    parser = ArgumentParser(prog='grasping_controller_interface.py', description='Change to run everything in just the demo mode.')
    parser.add_argument('--demo', default=False, help='Change execution to work with just the moveit demo mode.')
    parsed_args = parser.parse_known_args(sys.argv[1:])
    return parsed_args

# Controller part of step 3 and 4
def pick(sh_interface, gripper_interface):
    '''
    Picks the book specified in the given interfaces and holds it in front of
    the robots camera.

    :param SHInterface sh_interface: The initialized interface to interact with
        the shadow hand.
    :param GripperInterface gripper_interface: The initialized interface to
        interact with the gripper.

    :return bool: True if grasping was successful, False if not.
    '''
    if book_shelf == 1:
        grasp_lift = 0.192
    else:
        grasp_lift = 0.52
    # Step 3.1
    try:
        sh_interface.move_torso_shelf(book_shelf)
    except MoveItCommanderException as e:
        rospy.logerr(e)
        return False
    # Controller part of step 3.2
    response = sh_interface.tilt_book(lift = 0.1745)
    if not response.val == 1:
        return False
    # Controller part of step 3.3
    response = gripper_interface.grasp_book(gripper_interface.get_grasp_pose(height = 0.02, distance = 0.05), start_step = 0, lift = grasp_lift)
    if not response.val == 1:
        return False
    # Controller part of step 3.4.1 and 3.4.2
    response = sh_interface.move_from_book()
    if not response.val == 1:
        return False
    # Controller part of step 3.4.3 - 3.4.7
    response = gripper_interface.extract_book_regrasp()
    if not response.val == 1:
        return False
    # Step 4
    try:
        gripper_interface.move_book_camera()
    except MoveItCommanderException as e:
        rospy.logerr(e)
        return False
    return True

def pick_data(sh_interface, gripper_interface):
    '''
    Picks the book specified in the given interfaces and holds it in front of
    the robots camera.

    :param SHInterface sh_interface: The initialized interface to interact with
        the shadow hand.
    :param GripperInterface gripper_interface: The initialized interface to
        interact with the gripper.

    :return bool: True if grasping was successful, False if not.
    '''
    if book_shelf == 1:
        grasp_lift = 0.192
    else:
        grasp_lift = 0.52
    # Step 3.1
    try:
        sh_interface.move_torso_shelf(book_shelf)
    except MoveItCommanderException as e:
        rospy.logerr(e)
        return False
    # Controller part of step 3.2
    response = sh_interface.tilt_book(lift = 0.1745)
    if not response.val == 1:
        return False
    # Controller part of step 3.3
    response = gripper_interface.grasp_book(gripper_interface.get_grasp_pose(height = 0.02, distance = 0.05), start_step = 0, lift = grasp_lift)
    if not response.val == 1:
        return False
    # Controller part of step 3.4.1 and 3.4.2
    response = sh_interface.move_from_book()
    if not response.val == 1:
        return False
    # Controller part of step 3.4.3 - 3.4.7
    response = gripper_interface.extract_book_regrasp()
    if not response.val == 1:
        return False
    return True

# Controller part of step 5
def place(sh_interface, gripper_interface, place_pose, direction):
    '''
    Places the book currently attached to the gripper at the given place pose.

    :param GripperInterface gripper_interface: The initialized interface to
        interact with the gripper.
    :param PoseStamped place_pose: The pose where the book should be placed at.
    :param String direction: 'left' if the gripper should approach the place
        pose from the left, 'right' if from the right.

    :return bool: True if placing was successful, False if not.
    '''
    # Step 5.1
    try:
        sh_interface.move_torso_shelf(place_shelf)
    except MoveItCommanderException as e:
        rospy.logerr(e)
        return False
    # Controller part of step 5.2
    response = gripper_interface.place_book_vertical(place_pose, direction)
    if not response.val == 1:
        return False
    response = gripper_interface.push_book_upright(direction)
    if not response.val == 1:
        return False
    # Controller part of step 5.3
    try:
        sh_interface.move_torso_shelf(0)
    except MoveItCommanderException as e:
        rospy.logerr(e)
        return False
    return True

def pick_place(sh_interface, gripper_interface, place_pose, direction):
    '''
    Picks the book specified in the interfaces, holds it in front of the
    camera and places it at the specified place pose.

    :param SHInterface sh_interface: The initialized interface to interact with
        the shadow hand.
    :param GripperInterface gripper_interface: The initialized interface to
        interact with the gripper.
    :param PoseStamped place_pose: The pose where the book should be placed at.
    :param String direction: 'left' if the gripper should approach the place
        pose from the left, 'right' if from the right.

    :return bool: True if picking and placing was successful, False if not.
    '''
    response = pick(sh_interface, gripper_interface)
    if not response:
        return response
    return place(sh_interface, gripper_interface, place_pose, direction)

def manipulation_service(man_req):
    '''
    Processes the received manipulation request.

    :param Manipulate man_req: The manipulation request to be processed.

    :return bool: True if request was successful, False if not.
    '''
    global book_shelf, place_shelf
    req = man_req.request

    rospy.loginfo('Message received with command {}.'.format(req.command))

    placement = req.placement
    place_pose = placement.place_pose
    if placement.approach_left:
        direction = 'left'
    else:
        direction = 'right'
    book_id = 'book-{}'.format(placement.book.database_id)

    # Step 2
    sh_interface = SHInterface(general_controllers, right_arm_controllers, book_id = book_id, demo = demo, finger_flat = True)
    gripper_interface = GripperInterface(general_controllers, left_arm_controllers, book_id = book_id, demo = demo)

    # Check in which shelf the book is located
    book_pose = sh_interface.get_book_pose()
    if book_pose.pose.position.z > 0.948: # Check against height of upper shelf
        book_shelf = 1
    else:
        book_shelf = 0

    # Check in which shelf the book should be placed
    if place_pose.pose.position.z > 0.948: # Check against height of upper shelf
        place_shelf = 1
    else:
        place_shelf = 0

    if req.command == 'pick':
        return pick(sh_interface, gripper_interface)
    elif req.command == 'place':
        return place(sh_interface, gripper_interface, place_pose, direction)
    elif req.command == 'place_book':
        return pick_place(sh_interface, gripper_interface, place_pose, direction)
    elif req.command == 'pick_data':
        return pick_data(sh_interface, gripper_interface)
    else:
        return False

def main():
    global demo, general_controllers, left_arm_controllers, right_arm_controllers
    demo = parse_args()[0].demo
    demo = demo == 'True'

    roscpp_initialize(sys.argv)

    # Initialize controllers for both interfaces
    general_controllers.append(RobotCommander())
    general_controllers.append(PlanningSceneInterface(synchronous = True))
    general_controllers.append(MoveGroupCommander('torso', wait_for_servers = 10))

    # Initializes controllers for the gripper
    left_arm_controllers.append(MoveGroupCommander('left_arm'))
    left_arm_controllers.append(MoveGroupCommander('left_gripper'))

    # initializes controllers for the shadowhand
    right_arm_controllers.append(MoveGroupCommander('right_arm'))
    right_arm_controllers.append(MoveGroupCommander('right_fingers'))
    right_arm_controllers.append(MoveGroupCommander('right_arm_pr2'))
    right_arm_controllers.append(MoveGroupCommander('right_hand'))
    right_arm_controllers.append(MoveGroupCommander('right_arm_and_hand'))

    # Step 1 and 6
    rospy.init_node('grasping_controller')
    rospy.Service('manipulation_service', ManipulationService, manipulation_service)
    rospy.loginfo('Grasping controller started, with demo mode {}. Awaiting messages.'.format(demo))
    rospy.spin()

if __name__ == '__main__':
    main()
