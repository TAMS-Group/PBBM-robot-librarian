#!/usr/bin/env python3

#TODO: maybe filtering for the env node; 
#TODO: change the srv definition and add the wainting untill the result
from librarian_resources.srv import EnvironmentService,  ManipulationService, ToggleVisionNode   
from librarian_resources.msg import Environment, Book, BookRecognition, Manipulate, VisionControl
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

import rospy 
import rospkg

from copy import deepcopy
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import pandas as pd
import numpy as np

class Books:
    rospack = rospkg.RosPack()
    DATABASE_PATH = rospack.get_path('librarian_resources')+"/books.csv"
    DATABASE = pd.read_csv(DATABASE_PATH).set_index('id')
    #state of books, sorting them
    # def __init__(self, environment_msg: Environment):
    #     self.env = self.update_books(environment_msg)

    def update_books(self, environment_msg: Environment):
        self.world_state = self.props_from_database(environment_msg)


    def props_from_database(self, environment_msg: Environment):

        books_list = list(environment_msg.books)
        
        #add here whatever we might need to "sort" the books 
        #derive it later on 
        # data_to_derive = []#["color" ]
        
        #combine info from database with info from recognition
        columns = list(self.DATABASE.columns)
        columns.extend(["header","pose", "text", "spine", "book_msg"])#.extend(data_to_derive)
        #print(columns)
        world_state = pd.DataFrame(columns=columns)
        #print(world_state)
        for book in books_list:
            #print(book)
            # print(self.DATABASE)
            # print(self.DATABASE.loc[book.database_id])
            world_state = world_state.append(self.DATABASE.loc[book.database_id])
            world_state.at[book.database_id, "header"]      = environment_msg.header
            world_state.at[book.database_id, "pose"]        = book.recognition.pose
            # print([i.data for i in book.recognition.text])
            # world_state.at[book.database_id, "text"]        = [i.data for i in book.recognition.text]
            # world_state.at[book.database_id, "confidences"] = np.array(book.recognition.confidences)
            world_state.at[book.database_id, "spine"]       = book.recognition.spine
            world_state.at[book.database_id, "book_msg"]    = book

            # world_state.at[book.database_id, "color"] = 
        
        return world_state

    def sort_by(self, col, ascending=True):
        '''get a list of books sorted by the feature 
        with a corresponting column name in the df '''
        # print(self.world_state.sort_values(by=col, ascending=ascending)["book_msg"].tolist())
        #print(self.world_state.columns)
        return self.world_state.sort_values(by=col, ascending=ascending)["book_msg"].tolist()

    def book_props(self, id, col):
        return self.world_state.loc[id][col]

    def leftest_book(self):
        '''temporary func
        books: list of book msgs
        return: (x, y, z) of the leftest book (with the smallest y)
        get the leftest book, assuming that the books are 
        in the shelf frame and books are parallel'''

        def pos_x(pose_msg):
            return pose_msg.pose.position.x

        def key_sort(series):
            return series.apply(pos_x)

        print("check if sorting by the y is correct")
        return self.world_state.sort_values(by='pose', key=key_sort)["book_msg"].iloc[0]



class Interfaces:
    def __init__(self):
        #3 interfaces here: manipulation, env, perception
        #rospy.wait_for_service('environment_service')
        rospy.wait_for_service('manipulation_service')
        #rospy.wait_for_service('toggle_vision_node_service')

        #self.env_srv = rospy.ServiceProxy('environment_service', EnvironmentService)
        self.man_srv = rospy.ServiceProxy('manipulation_service', ManipulationService)
        #self.percept_srv = rospy.ServiceProxy('toggle_vision_node_service', ToggleVisionNode)
        self.percept_pub = rospy.Publisher('/librarian/control/vision', VisionControl, queue_size = 1)
    
    def pick_place(self, book, approach_side, place_pose):#place_pos, place_orient_quat,):
        '''book: instance of t'''
        # place_msg    
        # #book to manipulate
        # librarian_resources/Book book
        # #where to place the book
        # geometry_msgs/Pose place_pose
        # #approach the placing pose from the left
        # bool approach_left
        self.toggle_perception(False)
        man_srv_msg = Manipulate()
        man_srv_msg.header.stamp = rospy.Time.now()
        print(man_srv_msg)
        man_srv_msg.command = "place_book"

        man_srv_msg.placement.book = book
        man_srv_msg.placement.place_pose = place_pose
        # man_srv_msg.placement.place_pose = Pose()
        # man_srv_msg.placement.place_pose.position.x = place_pos[0]
        # man_srv_msg.placement.place_pose.position.y = place_pos[1]
        # man_srv_msg.placement.place_pose.position.z = place_pos[2]
        # man_srv_msg.placement.place_pose.orientation.w = place_orient_quat[0]
        # man_srv_msg.placement.place_pose.orientation.x = place_orient_quat[1]
        # man_srv_msg.placement.place_pose.orientation.y = place_orient_quat[2]
        # man_srv_msg.placement.place_pose.orientation.z = place_orient_quat[3]

        man_srv_msg.placement.approach_left = approach_side
        responce = self.man_srv(man_srv_msg)
        self.toggle_perception(True)

        return responce

    def place_book_lower_left(self, book, cm, command = 'place_book'):
        '''place the book on the lower shelf on the left side'''
                # place_msg    
        # #book to manipulate
        # librarian_resources/Book book
        # #where to place the book
        # geometry_msgs/Pose place_pose
        # #approach the placing pose from the left
        # bool approach_left
        #self.toggle_perception(False)
        man_srv_msg = Manipulate()
        man_srv_msg.header.stamp = rospy.Time.now()
        man_srv_msg.command = command

        man_srv_msg.placement.book = book
        # man_srv_msg.placement.place_pose = place_pose
        man_srv_msg.placement.place_pose = PoseStamped()
        man_srv_msg.placement.place_pose.header.frame_id = "shelf_0_0_0"
        man_srv_msg.placement.place_pose.pose.position.x = cm
        man_srv_msg.placement.place_pose.pose.position.y = 0.003
        man_srv_msg.placement.place_pose.pose.position.z = 0.003
        man_srv_msg.placement.place_pose.pose.orientation.w = 0
        man_srv_msg.placement.place_pose.pose.orientation.x = 0.7071068
        man_srv_msg.placement.place_pose.pose.orientation.y = 0.7071068
        man_srv_msg.placement.place_pose.pose.orientation.z = 0

        man_srv_msg.placement.approach_left = False
        print("+++++++++++++++++++++++++++++++++++++++++==")
        #print(man_srv_msg)
        responce = self.man_srv(man_srv_msg)
        #self.toggle_perception(True)

        return responce
        
    # def place_book_0_0_0_cm(self, book, cm):
    #     '''place the book on the lower shelf on the left side'''
    #             # place_msg    
    #     # #book to manipulate
    #     # librarian_resources/Book book
    #     # #where to place the book
    #     # geometry_msgs/Pose place_pose
    #     # #approach the placing pose from the left
    #     # bool approach_left
    #     self.toggle_perception(False)
    #     man_srv_msg = Manipulate()
    #     man_srv_msg.header.stamp = rospy.Time.now()
    #     man_srv_msg.command = "place_book"

    #     man_srv_msg.placement.book = book_to_place
    #     # man_srv_msg.placement.place_pose = place_pose

    #     man_srv_msg.placement.place_pose = book_left.pose

    #     man_srv_msg.placement.place_pose.position.x = cm


    #     man_srv_msg.placement.approach_left = False
    #     responce = self.man_srv(man_srv_msg)
    #     self.toggle_perception(True)

    #     return responce


    def shift_books(self, book, approach_left: bool):
        '''push the books to the side
            the command for the manipulation specified with a string that contains command,
            the book on which to push and the pushing direction
            '''
        
        # place_msg    
        # #book to manipulate
        # librarian_resources/Book book1 
        # #where to place the book
        # geometry_msgs/Pose place_pose
        # #approach the placing pose from the left
        # bool approach_left
        self.toggle_perception(False)
        man_srv_msg = Manipulate()
        man_srv_msg.header.stamp = rospy.Time.now()
        man_srv_msg.command = "shift_books"
        man_srv_msg.placement.book = book
        man_srv_msg.placement.approach_left = approach_left

        responce = self.man_srv(man_srv_msg)
        self.toggle_perception(True)
        return responce


    def update_env(self):
        return self.env_srv("update")

    def toggle_perception(self, flag):
        # return self.percept_srv(flag)
        self.percept_pub.publish(VisionControl(enabled = flag))

    # later: possibly define actions as functions for convinience. Depends on the interfaces with other nodes

class Interact:
    def __init__(self):
        #get world state, define services and topics to publish        
        self.inter = Interfaces()
        self.env = Books()   
        self.inter.toggle_perception(True)
        print("before update")
        self.update_env()
        print("after update")


    def update_env(self):
        # print(self.inter.update_env())
        # self.env.update_books(self.inter.update_env().status) #for the test case    
        self.env.update_books(rospy.wait_for_message("/librarian/environment", Environment))     # for the actual hardware


    #TODO: use *args, specify the parametrs of action
    def invoke_action(self, action):
        # method to use as a callback for the main 
        action = action.data
        if action == "sort":
            rospy.loginfo("sorting books")
            self.sort_books()
        elif action == "w":
            rospy.loginfo("sorting books")
            self.sort_books(target = "w")
        elif action == "r":
            rospy.loginfo("sorting books")
            self.sort_books(target = "r")
        elif action == "y":
            rospy.loginfo("sorting books")
            self.sort_books(target = "y")
        elif action == "fetch a book":
            self.fetch_book()
    

    def sort_books_demo_moveit(self):
        '''small demo to sort books one one shelf
        assumptions: books are on one shelf; enough of space 
        for now the pose of the books hould be in the shelf frame
        on the shelf to manipulate them
        books are in on the right side of the shelf'''
        EPS = -0.0001

        self.update_env()
        books_ordered = self.env.sort_by("height")
        for book in books_ordered:
            leftest_book = self.env.leftest_book()

            place_pose = deepcopy(leftest_book.recognition.pose)
            # as agreed with Bjoern, specify the place pose by the lowest, shalest corner
            # and approach direction
            place_pose.pose.position.x -= self.env.book_props(leftest_book.database_id, "depth")/2 - EPS
            place_pose.pose.position.y = 0
            place_pose.pose.position.z -= self.env.book_props(leftest_book.database_id, "height")/2
            assert self.inter.pick_place(book, True,place_pose).status ==0, "manipulation failed"
            
            self.update_env()
            leftest_book = self.env.leftest_book()
            
            self.inter.shift_books(leftest_book, True)
            self.update_env()

    def sort_books(self, target = 'default'):
        '''small demo to sort books one one shelf
        assumptions: books are on one shelf; enough of space 
        for now the pose of the books hould be in the shelf frame
        on the shelf to manipulate them
        books are in on the right side of the shelf'''

        self.update_env()
        self.inter.toggle_perception(False)
        rospy.sleep(2)
        books_ordered = self.env.sort_by("height", ascending=False)
        books_ordered = books_ordered[1:]
        '''
        for book in books_ordered:
            print(self.env.book_props(book.database_id, "height"))
            print(book.database_id)
        return
        '''
        cm_counter = 0
        book: Book
        if target == 'default':
            for book in books_ordered:
                print(book.database_id)
                assert self.inter.place_book_lower_left(book, cm_counter).status == True, "manipulation failed"
                cm_counter += self.env.book_props(book.database_id, "depth")
                # as agreed with Bjoern, specify the place pose by the lowest, shalest corner
                #self.update_env()
            self.inter.toggle_perception(True)
        else:
            if target == 'w':
                self.inter.place_book_lower_left(books_ordered[0], cm_counter, command = 'pick_data')
            if target == 'r':
                self.inter.place_book_lower_left(books_ordered[1], cm_counter, command = 'place_book')
            if target == 'y':
                self.inter.place_book_lower_left(books_ordered[2], cm_counter, command = 'pick_data')
        rospy.loginfo("Finished sort request.")


    def fetch_book(self, book):
        pass


if __name__ == '__main__':
    rospy.init_node('interaction_node', anonymous=True)
    interact = Interact()
    print("X")
    rospy.Subscriber("chatter", String, interact.invoke_action)
    rospy.spin()




# "sort books"
# assumptions: the books are on one shelf, there is enough of the space to manipulate them


# the graping and placing are specified in one mgs:"Manipulation" pick+place string as a command for example
#     for the grasping: just name of the book is needed
#     for the palcing: side of the approach, lowest point and furthest from the approach direction of the contact


# for the testing the shifting can be implemented just by shifting all poses relative to the shelf by the length of the manipulated book
# todo: add a method to rearange the random init into a standard scene setup



# 1. make a subscriber that handles the requests and select the desired "sorting"
# 2. the desired sorting performs a procedurally generated sequence of actions
#     a. push the books to the right
#     b. percieve the env
#     c. sort the books in the env by the desired parameter (output: sorted list)

#     for book in the sorted_list:
#         i.  grasp the book in order of the list and apend to the begining of the shelf
#         ii. push the books to the right   
#         iii.  update the env
#         TODO: handle the errors as: cannot pick and cannot place

# need to implment:
#     subscriber that select the desired behavior
#     fake "shifting" of the books in the test node
#     how to manipulate the world: some procedural generation of appending books from the left to the book
#         for now use the pandas frame with the assumption that books are on the same shelf 
#         (later, we can split the frame on the diffenet shelf for example)
#         and then sort the by the x coordinate. this will be the order of the books
