#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
import trajectory_msgs.msg
import librarian_resources.msg
import sys
from moveit_commander.exception import MoveItCommanderException

torso = None
left_arm = None
left_gripper = None
right_arm = None
right_arm_and_hand = None
right_fingers = None
scene = None

def reset_left_arm():
    try:
        left_arm.set_named_target('left_arm_to_side')
        left_arm.go(wait = True)
        left_arm.stop()
        left_gripper.set_named_target('open')
        left_gripper.go(wait = True)
        left_gripper.stop()
    except MoveItCommanderException:
        raise

def reset_right_arm():
    try:
        right_arm.set_named_target('right_arm_to_side')
        right_arm.go(wait = True)
        right_arm.stop()
        right_fingers.set_joint_value_target(right_fingers.get_current_joint_values())
        right_fingers.set_joint_value_target('rh_FFJ4', 0)
        right_fingers.set_joint_value_target('rh_FFJ3', 0)
        right_fingers.set_joint_value_target('rh_FFJ2', 0.0175)
        right_fingers.set_joint_value_target('rh_MFJ4', 0)
        right_fingers.set_joint_value_target('rh_MFJ3', 0)
        right_fingers.set_joint_value_target('rh_MFJ2', 0.0175)
        right_fingers.set_joint_value_target('rh_RFJ4', 0)
        right_fingers.set_joint_value_target('rh_RFJ3', 0)
        right_fingers.set_joint_value_target('rh_RFJ2', 0.0175)
        right_fingers.set_joint_value_target('rh_LFJ5', 0.0175)
        right_fingers.set_joint_value_target('rh_LFJ4', 0)
        right_fingers.set_joint_value_target('rh_THJ5', 0)
        right_fingers.set_joint_value_target('rh_THJ4', 0.0175)
        right_fingers.set_joint_value_target('rh_THJ3', 0)
        right_fingers.set_joint_value_target('rh_THJ2', 0)
        right_fingers.go(wait = True)
        right_fingers.stop()
        right_arm_and_hand.set_joint_value_target(right_arm_and_hand.get_current_joint_values())
        right_arm_and_hand.set_joint_value_target('rh_WRJ1', 0)
        right_arm_and_hand.set_joint_value_target('rh_WRJ2', 0)
        right_arm_and_hand.go(wait = True)
        right_arm_and_hand.stop()
    except MoveItCommanderException:
        raise

def delete_book(book):
    '''
    Deletes the collision object book from the moveit scene.
    The object gets deleted whether it is attached or not.
    '''
    if book in scene.get_objects():
        scene.remove_world_object(book)
        rospy.loginfo('Deleted the world collision object {}.'.format(book))
    elif book in scene.get_attached_objects():
        scene.remove_attached_object(name=book)
        scene.remove_world_object(book)
        rospy.loginfo('Deleted the attached collision object {}.'.format(book))
    else:
        rospy.loginfo('No object with the name {} was found in the planning scene. No object was deleted.'.format(book))

def main():
    global torso, left_arm, left_gripper, scene, right_arm, right_arm_and_hand, right_fingers

    rospy.init_node('reset_arms')
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface(synchronous = True)
    torso = moveit_commander.MoveGroupCommander('torso')
    torso.set_max_velocity_scaling_factor(0.9)
    torso.set_max_acceleration_scaling_factor(0.9)
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    left_gripper = moveit_commander.MoveGroupCommander('left_gripper')
    right_arm = moveit_commander.MoveGroupCommander('right_arm_pr2')
    right_arm_and_hand = moveit_commander.MoveGroupCommander('right_arm')
    right_fingers = moveit_commander.MoveGroupCommander('right_fingers')

    rospy.loginfo('Resetting right arm.')
    reset_right_arm()
    rospy.loginfo('Resetting left arm.')
    reset_left_arm()
    if sys.argv[1] == 'w':
        book = 'book-3'
    elif sys.argv[1] == 'y':
        book = 'book-4'
    elif sys.argv[1] == 'r':
        book = 'book-1'
    else:
        book = 'book'
    delete_book(book)

    rospy.loginfo('Resetting torso.')
    torso.set_joint_value_target([0.016])
    torso.go(wait = True)

if __name__ == '__main__':
    main()
