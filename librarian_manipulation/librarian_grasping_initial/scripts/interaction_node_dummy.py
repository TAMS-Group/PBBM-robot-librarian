#!/usr/bin/env python3

import rospy
import sys
from librarian_resources.srv import ManipulationService
from librarian_resources.msg import Manipulate
from geometry_msgs.msg import PoseStamped
from moveit_commander import roscpp_initialize, PlanningSceneInterface

def main():
    # Initialize
    roscpp_initialize(sys.argv)
    scene = PlanningSceneInterface(synchronous = True)
    rospy.wait_for_service('manipulation_service')
    man_srv = rospy.ServiceProxy('manipulation_service', ManipulationService)
    rospy.loginfo('Manipulation service connected.')
    books = [15, 17]
    shift = 0

    # Spawn all specified books.
    for book in books:
        # Prepare manipulation message
        man_srv_msg = Manipulate()
        man_srv_msg.command = "place_book"
        man_srv_msg.placement.book.database_id = book
        man_srv_msg.placement.place_pose = PoseStamped()
        man_srv_msg.placement.place_pose.header.frame_id = "shelf_0_0_0"
        man_srv_msg.placement.place_pose.pose.position.x = shift
        man_srv_msg.placement.place_pose.pose.position.y = 0.003
        man_srv_msg.placement.place_pose.pose.position.z = 0.003
        man_srv_msg.placement.place_pose.pose.orientation.x = 0.7071068
        man_srv_msg.placement.place_pose.pose.orientation.y = 0.7071068
        man_srv_msg.placement.place_pose.pose.orientation.z = 0
        man_srv_msg.placement.place_pose.pose.orientation.w = 0
        man_srv_msg.placement.approach_left = False

        # Send manipulation message
        rospy.loginfo('Sending manipulation message.')
        if not man_srv(man_srv_msg):
            rospy.loginfo('Manipulation failed.')
            return

        # Adjust place pose shift to compensate for last created book
        book_object = scene.get_objects([book])[book]
        book_dimensions = book_object.primitives[0].dimensions
        shift += book_dimensions[1]

if __name__ == '__main__':
    main()
