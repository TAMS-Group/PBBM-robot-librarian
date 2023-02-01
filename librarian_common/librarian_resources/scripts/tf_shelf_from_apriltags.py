#!/usr/bin/env python3
import rospy

import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from collections import deque

import tf_conversions

import tf2_ros
import geometry_msgs.msg
from librarian_resources.msg import BookShelfInfo 

import PyKDL as kdl

poses_for_average = deque(maxlen=20)

#TODO: make a roslaunch with the param size for the apriltag
#and detection with tf_broadcasting node
#fix the book spawn

tag_size=0.144*10/8
camera_frame=rospy.get_param("/librarian/source_frame", "azure_kinect_rgb_camera_link")

def call_back(msg_tags, pub_book_shelf_info):
    global pub
    pub_book_shelf_info = pub_book_shelf_info[0]
    br = tf2_ros.TransformBroadcaster()

    markers=list(msg_tags.detections)
    # print(markers[0])
    poses=dict(zip([i.id[0] for i in markers], [((i.pose.pose.pose.position.x, i.pose.pose.pose.position.y, i.pose.pose.pose.position.z),
        (i.pose.pose.pose.orientation.x, i.pose.pose.pose.orientation.y, i.pose.pose.pose.orientation.z, i.pose.pose.pose.orientation.w), i.pose.header.stamp) for i in markers]))

    book_shelf_info = BookShelfInfo(detections = msg_tags)

    try:        
        poses_for_average.append(np.array(poses[10]))
        T=kdl.Frame(kdl.Rotation.Quaternion(*np.average(np.array([i[1] for i in poses_for_average]), axis=0)), 
            kdl.Vector(*np.average(np.array([i[0] for i in poses_for_average]), axis=0)))
        
        #TODO: Maybe re-center the stl
        T_marker_to_shelf=kdl.Frame(kdl.Rotation.EulerZYX(np.pi/2,np.pi/2,0)*kdl.Rotation.EulerZYX(np.pi/2,0,0), 
            kdl.Vector((0.765+0.016*2-tag_size/2), -1.25+tag_size/2, -(0.36+0.016)))
            # -(0.36+0.016),-(0.765+0.016*2) , -1.25)

        T_shelf=T*T_marker_to_shelf
        
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = camera_frame
        t.child_frame_id = "shelf" 
        t.header.stamp = poses[10][2]

        t.transform.translation.x = T_shelf.p[0]
        t.transform.translation.y = T_shelf.p[1]
        t.transform.translation.z = T_shelf.p[2]

        rotation = np.array(T_shelf.M.GetQuaternion())
        rotation = rotation / np.sqrt((rotation**2).sum())
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        # print "shelf frame: ", str(t)
        br.sendTransform(t)



        #upper shelf left bottom
        #notation of the shelfs are following the convention:
        #<shelf_name>_<shelf_level(from 0)>_<left(0)/right(1)>_<bottom(0)/top(1)>

        T_shelf_to_shelf_1_0_0=kdl.Frame(kdl.Rotation.EulerZYX(0,0,-np.pi/2), 
            kdl.Vector((0-tag_size/2+0.016), -(1.25-1.235)-0.287+tag_size/2, 0))
        T_shelf_1_0_0=T*T_shelf_to_shelf_1_0_0
        
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = camera_frame
        t.child_frame_id = "shelf_1_0_0" 
        t.header.stamp = poses[10][2]

        t.transform.translation.x = T_shelf_1_0_0.p[0]
        t.transform.translation.y = T_shelf_1_0_0.p[1]
        t.transform.translation.z = T_shelf_1_0_0.p[2]


        rotation = np.array(T_shelf_1_0_0.M.GetQuaternion())
        rotation = rotation / np.sqrt((rotation**2).sum())
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        br.sendTransform(t)
        book_shelf_info.shelf_mid_left = t

        #upper shelf right lower corner
        T_shelf_1_0_0_to_T_shelf_1_1_0=kdl.Frame(kdl.Rotation.EulerZYX(0,0,0), 
            kdl.Vector((0+0.765),0,0))
        T_shelf_1_1_0 = T_shelf_1_0_0*T_shelf_1_0_0_to_T_shelf_1_1_0
        
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = camera_frame
        t.child_frame_id = "shelf_1_1_0" 
        t.header.stamp = poses[10][2]

        t.transform.translation.x = T_shelf_1_1_0.p[0]
        t.transform.translation.y = T_shelf_1_1_0.p[1]
        t.transform.translation.z = T_shelf_1_1_0.p[2]


        rotation = np.array(T_shelf_1_1_0.M.GetQuaternion())
        rotation = rotation / np.sqrt((rotation**2).sum())
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        br.sendTransform(t)
        book_shelf_info.shelf_mid_right = t

        #upper shelf right upper corner
        T_shelf_1_0_0_to_T_shelf_1_1_1=kdl.Frame(kdl.Rotation.EulerZYX(0,0,0), 
            kdl.Vector((0+0.765),0,0.287))
        T_shelf_1_1_1 = T_shelf_1_0_0*T_shelf_1_0_0_to_T_shelf_1_1_1
                        
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = camera_frame
        t.child_frame_id = "shelf_1_1_1" 
        t.header.stamp = poses[10][2]

        t.transform.translation.x = T_shelf_1_1_1.p[0]
        t.transform.translation.y = T_shelf_1_1_1.p[1]
        t.transform.translation.z = T_shelf_1_1_1.p[2]


        rotation = np.array(T_shelf_1_1_1.M.GetQuaternion())
        rotation = rotation / np.sqrt((rotation**2).sum())
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        br.sendTransform(t)
        book_shelf_info.shelf_top_right = t

        #upper shelf left upper corner
        T_shelf_1_0_0_to_T_shelf_1_0_1=kdl.Frame(kdl.Rotation.EulerZYX(0,0,0), 
            kdl.Vector(0,0,0.287))
        T_shelf_1_0_1 = T_shelf_1_0_0*T_shelf_1_0_0_to_T_shelf_1_0_1
        
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = camera_frame
        t.child_frame_id = "shelf_1_0_1" 
        t.header.stamp = poses[10][2]

        t.transform.translation.x = T_shelf_1_0_1.p[0]
        t.transform.translation.y = T_shelf_1_0_1.p[1]
        t.transform.translation.z = T_shelf_1_0_1.p[2]


        rotation = np.array(T_shelf_1_0_1.M.GetQuaternion())
        rotation = rotation / np.sqrt((rotation**2).sum())
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        br.sendTransform(t)
        book_shelf_info.shelf_top_left = t





        #lower shelf left lower corner
        T_shelf_1_0_0_to_shelf_0_0_0=kdl.Frame(kdl.Rotation.EulerZYX(0,0,-0), 
            kdl.Vector(0,0, -0.016-0.335))
        T_shelf_0_0_0=T_shelf_1_0_0*T_shelf_1_0_0_to_shelf_0_0_0
        
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = camera_frame
        t.child_frame_id = "shelf_0_0_0"
        t.header.stamp = poses[10][2]

        t.transform.translation.x = T_shelf_0_0_0.p[0]
        t.transform.translation.y = T_shelf_0_0_0.p[1]
        t.transform.translation.z = T_shelf_0_0_0.p[2]


        rotation = np.array(T_shelf_0_0_0.M.GetQuaternion())
        rotation = rotation / np.sqrt((rotation**2).sum())
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        br.sendTransform(t)
        book_shelf_info.shelf_bot_left = t


        #lower shelf right lower corner
        T_shelf_0_0_0_to_T_shelf_0_1_0=kdl.Frame(kdl.Rotation.EulerZYX(0,0,0), 
            kdl.Vector((0+0.765),0,0))
        T_shelf_0_1_0 = T_shelf_0_0_0*T_shelf_0_0_0_to_T_shelf_0_1_0
        
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = camera_frame
        t.child_frame_id = "shelf_0_1_0" 
        t.header.stamp = poses[10][2]

        t.transform.translation.x = T_shelf_0_1_0.p[0]
        t.transform.translation.y = T_shelf_0_1_0.p[1]
        t.transform.translation.z = T_shelf_0_1_0.p[2]


        rotation = np.array(T_shelf_0_1_0.M.GetQuaternion())
        rotation = rotation / np.sqrt((rotation**2).sum())
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        br.sendTransform(t)
        book_shelf_info.shelf_bot_right = t

        #lower shelf right upper corner
        T_shelf_0_0_0_to_T_shelf_0_1_1=kdl.Frame(kdl.Rotation.EulerZYX(0,0,0), 
            kdl.Vector((0+0.765),0,0.335))
        T_shelf_0_1_1 = T_shelf_0_0_0*T_shelf_0_0_0_to_T_shelf_0_1_1
        
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = camera_frame
        t.child_frame_id = "shelf_0_1_1" 
        t.header.stamp = poses[10][2]

        t.transform.translation.x = T_shelf_0_1_1.p[0]
        t.transform.translation.y = T_shelf_0_1_1.p[1]
        t.transform.translation.z = T_shelf_0_1_1.p[2]


        rotation = np.array(T_shelf_0_1_1.M.GetQuaternion())
        rotation = rotation / np.sqrt((rotation**2).sum())
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        br.sendTransform(t)

        #lower shelf left upper corner
        T_shelf_0_0_0_to_T_shelf_0_0_1=kdl.Frame(kdl.Rotation.EulerZYX(0,0,0), 
            kdl.Vector(0,0,0.335))
        T_shelf_0_0_1 = T_shelf_0_0_0*T_shelf_0_0_0_to_T_shelf_0_0_1
        
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = camera_frame
        t.child_frame_id = "shelf_0_0_1" 
        t.header.stamp = poses[10][2]

        t.transform.translation.x = T_shelf_0_0_1.p[0]
        t.transform.translation.y = T_shelf_0_0_1.p[1]
        t.transform.translation.z = T_shelf_0_0_1.p[2]


        rotation = np.array(T_shelf_0_0_1.M.GetQuaternion())
        rotation = rotation / np.sqrt((rotation**2).sum())
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        br.sendTransform(t)

        book_shelf_info.header = rospy.Header()
        pub_book_shelf_info.publish(book_shelf_info)

    except KeyError:
        print("one of the markers was not detected")
        

    # pub.publish(generate_calibrated_state(msg_sensor, msg_joint_states)
    



if __name__ == '__main__':
    print("***")


    rospy.init_node('brodcast_camera_tf_from_tags')
    pub_book_shelf_info = rospy.Publisher("/book_shelf_info", BookShelfInfo, queue_size=2)
    rospy.Subscriber("tag_detections", AprilTagDetectionArray, call_back, callback_args=[pub_book_shelf_info], queue_size=10)
    rospy.spin()
