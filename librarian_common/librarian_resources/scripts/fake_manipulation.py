#!/usr/bin/env python3

from librarian_resources.msg import Environment, Book, BookRecognition, Manipulate
import rospy

from librarian_resources.srv import ManipulationService, ManipulationServiceResponse
                      
def manipulate_book(manipulate_msg: Manipulate):
    '''
    simulate the manipulation of the book
    '''
    rospy.logout("Manipulation is requested")
    rospy.logout(manipulate_msg)
    rospy.logout("after performing manipulation press enter")
    input()
    rospy.sleep(1.5)
    return 0


def main():
    rospy.init_node('fake_manipulation')
    manipulation_service = rospy.Service('manipulation_service', ManipulationService, manipulate_book)
    rospy.spin()

if __name__ == "__main__":
    main()