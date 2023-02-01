#!/usr/bin/env python3
# script that simulates interaction with env node
# Note, that book rotated with respect to the dimention names in dataset

from librarian_resources.msg import Environment, Book, BookRecognition, Manipulate
from std_msgs.msg import String
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped
import moveit_commander

import rospy
import rospkg

from librarian_resources.srv import EnvironmentService, EnvironmentServiceResponse,\
                                    ManipulationService, ManipulationServiceResponse,\
                                    ToggleVisionNode, ToggleVisionNodeResponse



import pandas as pd


class EnvironmentSim:
    '''
    a class to model and simulate interaction with environment
    for now: the obects are stored and manipulated ditecly as a corresponding msgs 
    '''
    #the world model is the list of the books recognized
    #therefore the world model is a dict of the recognized books
    #
    #the world can be manipulated by changing the book properties
    #so all manipualtions are done by using this dict
    
    books = {}
    perception_allowed = True
    scene = None
    rospack = rospkg.RosPack()
    DATABASE_PATH = rospack.get_path('librarian_resources')+"/books.csv"
    DATABASE = pd.read_csv(DATABASE_PATH).set_index('id')
    SHELF_LENGTH = 0.765 # + 0.016
    rospy.logwarn("Why does the shelf origin does not coincide with the model")

    def __init__(self):
        
        list_books = []
        list_books.append(self.create_book_msg(database_id=1, pos_x=0.50, text_recoginzed = "book 1 sample text"))
        list_books.append(self.create_book_msg(database_id=2, pos_x=0.54, text_recoginzed = "book 2 sample text"))
        # list_books.append(self.create_book_msg(database_id=3, pos_x=0.15, text_recoginzed = "book 3 sample text"))
        # list_books.append(self.create_book_msg(database_id=4, pos_x=0.22, text_recoginzed = "book 4 sample text"))
        # list_books.append(self.create_book_msg(database_id=5, pos_x=0.30, text_recoginzed = "book 5 sample text"))
        # list_books.append(self.create_book_msg(database_id=6, pos_x=0.38, text_recoginzed = "book 6 sample text"))
        # list_books.append(self.create_book_msg(database_id=7, pos_x=0.44, text_recoginzed = "book 7 sample text"))
        # list_books.append(self.create_book_msg(database_id=8, pos_x=0.53, text_recoginzed = "book 8 sample text"))


        for book in list_books:
            self.books[book.database_id] = book

        env_msg = Environment()
        env_msg.books = list_books

        moveit_commander.roscpp_initialize([])
        self.scene = moveit_commander.PlanningSceneInterface(synchronous = True)
        self.create_shelf()
        self.planning_scene_update_all_books()



        ################# Testing code
        # man_msg = Manipulate()
        # man_msg.command = "shift_books"
        # man_msg.placement.approach_left = True
        # rospy.sleep(0.8)
        # self.manipulate_book(man_msg)

        # man_msg = Manipulate()
        # man_msg.command = "place_book"
        # man_msg.placement.approach_left = False
        # book_pose =  PoseStamped()
        # book_pose.header.frame_id = 'shelf_0_0_0'
        # book_pose.pose.orientation.w = 0#.7071068
        # book_pose.pose.orientation.z = 0#.7071068
        # book_pose.pose.position.x = 0 
        # man_msg.placement.place_pose = book_pose
        # man_msg.placement.book.database_id = 1
        # rospy.sleep(0.8)
        # self.manipulate_book(man_msg)
        
        # self.planning_scene_update_all_books()


    def create_book_msg(self, database_id, pos_x, text_recoginzed = "abc"):
        book_msg = Book()
        book_msg.database_id = database_id
        
        #create a book rec message
        book_rec = BookRecognition()
        book_rec.confidences = [0.1]
        
        text = String()
        text.data = text_recoginzed
        book_rec.text = [text]
        
        book_pose =  PoseStamped()
        book_pose.header.frame_id = 'shelf_0_0_0'
        book_pose.pose.orientation.w = 0.7071068
        book_pose.pose.orientation.z = 0.7071068
        book_pose.pose.position.x = pos_x 
        book_pose.pose.position.y = self.get_book_size(database_id)[1]/2 + 0.03
        book_pose.pose.position.z = self.get_book_size(database_id)[2]/2+0.003

        book_rec.pose = book_pose
        
        book_msg.recognition = book_rec

        return book_msg

    def get_book_size(self, database_id: int):
        # book_dimensions = (self.DATABASE.loc[database_id]["width"],
        #             self.DATABASE.loc[database_id]["depth"], #x
        #             self.DATABASE.loc[database_id]["height"])#z
        book_dimensions = (self.DATABASE.loc[database_id]["depth"], #x
                    self.DATABASE.loc[database_id]["width"],#y
                    self.DATABASE.loc[database_id]["height"])#z
        return book_dimensions

# ####################################################
# planning scene viz methods
    def planing_scene_create_book(self, book_msg: Book):
        book_pose = book_msg.recognition.pose
        # book_pose.header.frame_id = 'shelf_0_0_0'
        # book_pose.pose.orientation.w = 0.7071068
        # book_pose.pose.orientation.z = 0.7071068
        
        book_id = book_msg.database_id
        book_name = self.DATABASE.loc[book_id]["title"]
        book_dimensions = (self.DATABASE.loc[book_id]["width"], #x
                            self.DATABASE.loc[book_id]["depth"],#y
                            self.DATABASE.loc[book_id]["height"])#z
        
        self.scene.add_box('book-{}'.format(book_id), book_pose, size=book_dimensions)

    def planing_scene_delete_book(self, book_msg: Book):
        book_id = book_msg.database_id
        book_name = self.DATABASE.loc[book_id]["title"]
        if book_name in self.scene.get_objects():
            self.scene.remove_world_object(book_name)
        elif book_name in self.scene.get_attached_objects():
            self.scene.remove_attached_object(name=book_name)
            self.scene.remove_world_object(book_name)
            rospy.loginfo('Deleted the attached collision object book.')
        else:
            rospy.loginfo('{} was not deleted as it doesnt exist'.format(book_name))

    def planning_scene_update_book(self, book):
        self.planing_scene_delete_book(book)
        self.planing_scene_create_book(book)

    def planning_scene_update_all_books(self):
        rospy.loginfo("updating scene planning scene")
        for _, book in self.books.items():
            # print(book)
            self.planning_scene_update_book(book)

    def create_shelf(self, shelf_pose = None):
        if shelf_pose == None:
            shelf_pose = PoseStamped()
            shelf_pose.header.frame_id = "shelf"

        rospack = rospkg.RosPack()
        lib_res_path = rospack.get_path('librarian_resources')
        self.scene.add_mesh("shelf", shelf_pose, lib_res_path + '/meshes/shelf.stl')
        rospy.loginfo('Shelf was created.')

# ########################################################################
# dummy service methods

    def manipulate_book(self,  manipulate_msg: Manipulate):
        '''
        simulate the manipulation of the book
        '''
        EPS = 0.0001#dist between books when shifted

        manipulate_msg = manipulate_msg.request

        known_commands = set(["place_book", "shift_books"])
        if not str(manipulate_msg.command) in known_commands:
            rospy.logwarn("'{}' is not in known manipulation commands".format(manipulate_msg.request.command))

        if manipulate_msg.command =="place_book":
            rospy.logwarn("placing book")
            book_pose = manipulate_msg.placement.place_pose
            book_msg = manipulate_msg.placement.book
            book_id =book_msg.database_id
            book_dimensions = (self.DATABASE.loc[book_id]["depth"], #x
                                self.DATABASE.loc[book_id]["width"],#y
                                self.DATABASE.loc[book_id]["height"])#z
            
            book_pose.pose.position.z += book_dimensions[2]/2
            book_pose.pose.position.x += ((-1)**manipulate_msg.placement.approach_left)*book_dimensions[0]/2
            book_pose.pose.position.y += book_dimensions[1]/2

            rospy.logwarn("do we specify the center of the book or the center of the spine? use 2nd")
            # book_pose.position.x += book_dimensions[0]/2
            self.books[manipulate_msg.placement.book.database_id].recognition.pose = book_pose

        if manipulate_msg.command =="shift_books":
            # work in the shelf coordinate system
            # then, (0,0,0) is the shelf ?left? corner, (0,shelf_lenth,0) is the right corner
            # so we can sort book by the position y and reassing y coordinate, so the book will
                # be densly placed on the shelf starting form the desired side
            rospy.loginfo("shifting books")
            sign = (-1)**manipulate_msg.placement.approach_left
            if manipulate_msg.placement.approach_left == True:
                lenght_counter = self.SHELF_LENGTH
            else:
                lenght_counter = 0
            
            # print(self.books)
            keys_books_sorted_x = sorted(self.books.items(), key=lambda key_book: sign*key_book[1].recognition.pose.pose.position.x)
            for key_book in keys_books_sorted_x:
                key = key_book[0]
                self.books[key].recognition.pose.pose.position.x = lenght_counter + sign*self.DATABASE.loc[key]["depth"]/2 + sign*EPS
                # print( self.books[key].recognition.pose.pose.position.x )
                lenght_counter += sign*self.DATABASE.loc[key]["depth"] + sign*EPS
                # rospy.logwarn("shifting")
                self.planning_scene_update_all_books()
    
        self.planning_scene_update_all_books()
        rospy.sleep(1.5)
        return 0

    def get_env_state(self, request: str):
        rospy.loginfo("updating env repr")
        env_msg = Environment()
        env_msg.books = list(self.books.values())
        env_msg.header.stamp = rospy.Time.now()
        return env_msg

    def toggle_perception(self, flag):
        rospy.loginfo("setting allow perception {}".format(flag.on))
        self.perception_allowed = flag.on  
        resp = ToggleVisionNodeResponse(0)
        return resp

# ############################################################################
# ############################################################################


def main():

    rospy.init_node('service_server')
    env = EnvironmentSim()
    environment_service  = rospy.Service('environment_service',  EnvironmentService,  env.get_env_state)
    # manipulation_service = rospy.Service('manipulation_service', ManipulationService, env.manipulate_book)
    perception_service   = rospy.Service('toggle_vision_node_service',   ToggleVisionNode,   env.toggle_perception)
    rospy.spin()

if __name__ == "__main__":
    main()

