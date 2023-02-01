#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import MoveItErrorCodes, DisplayRobotState
from sensor_msgs.msg import JointState
from bio_ik_msgs.msg import JointVariableGoal, PositionGoal, DirectionGoal, IKRequest
from bio_ik_msgs.srv import GetIK
from geometry_msgs.msg import Quaternion, PoseStamped, Vector3, Point
from moveit_commander.exception import MoveItCommanderException
from tf.transformations import quaternion_multiply, quaternion_from_euler
from visualization_msgs.msg import Marker
from copy import deepcopy

class SHInterface():

    def __init__(self, general_controllers, arm_controllers, book_id = 'book', demo = False, finger_flat = False):
        self.robot = general_controllers[0]
        self.scene = general_controllers[1]
        self.torso = general_controllers[2]
        self.torso.set_max_velocity_scaling_factor(0.9)
        self.torso.set_max_acceleration_scaling_factor(0.9)
        self.arm_and_hand = arm_controllers[0]
        self.fingers = arm_controllers[1]
        self.arm = arm_controllers[2]
        self.hand = arm_controllers[3]
        self.arm_and_fingers = arm_controllers[4]
        self.base_frame = 'base_footprint'
        self.book_id = book_id
        self.demo = demo
        self.debug_pub = rospy.Publisher('~debug_sh_interface', PoseStamped, queue_size = 1, latch = True)
        self.visualize = True
        self.finger_flat = finger_flat

    # Interface part of step 3.2
    def tilt_book(self, lift, start_step = 0):
        '''
        Tilts the book in the shelf to be ready to be grasped by the gripper.

        :param float64 lift: A contraint for the right shoulder lift for the
            bio ik solver when moving to the book.
        :param int32 start_step: The step of this process which should be used
            as the starting step.

        :return MoveItErrorCodes: Corresponding error code to the success or
            failure of this process.
        '''
        current_step = start_step
        try:
            self.scene.enable_collision_detections(self.book_id, ['rh_ffdistal'])
        except:
            pass
        try:
            book_pose = self.get_book_pose()
            book_dimensions_halved = self.get_book_dimensions(halved = True)
            # Interface part of step 3.2.1
            if current_step == 0:
                if self.finger_flat:
                    rospy.loginfo('Moving fingers into pick position.')
                    self.move_ff_flat()
                else:
                    rospy.loginfo('Moving ff into hook position.')
                    self.move_ff_hook()
                current_step = 1
            # Step 3.2.2
            if current_step == 1:
                rospy.loginfo('Moving before initial position.')
                position = Point()
                position.x += book_pose.pose.position.x - book_dimensions_halved[0] + 0.0075 - 0.1# book pose x position - half of book depth + biotac height
                position.y += book_pose.pose.position.y #0.015 # book pose y position
                position.z += book_pose.pose.position.z + book_dimensions_halved[2] + 0.02# book pose z position + half of book height + offset
                self.move_position_bio_ik(position, direction = False, lift = lift)
                current_step = 2
            # Step 3.2.3
            if current_step == 2:
                rospy.loginfo('Moving into the initial position to tilt the book.')
                position = Point()
                position.x += book_pose.pose.position.x - book_dimensions_halved[0] + 0.0075 # book pose x position - half of book depth + biotac height
                position.y += book_pose.pose.position.y - 0.0008#biotac0.015 # book pose y position
                position.z += book_pose.pose.position.z + book_dimensions_halved[2] + 0.025# book pose z position + half of book height + offset
                self.move_position_bio_ik(position, direction = True, lift = lift)
                current_step = 3
            # Step 3.2.4
            if current_step == 3:
                rospy.loginfo('Moving downward just before making contact with the book.')
                self.move_upward(-0.03, tolerance = 0.1) # adjustment from 0.03 -> 0.025
                current_step = 4
            # Step 3.2.5
            if current_step == 4:
                rospy.loginfo('Moving downward a bit to make contact with the book.')
                self.scene.disable_collision_detections(self.book_id, ['rh_ffdistal'])
                self.move_upward(-0.025, tolerance = 0.1)#-0.015)#-0.035) #adjustment because wrong numbers or calibration error, needs to be tested
                self.arm_and_fingers.attach_object(self.book_id, link_name = 'rh_palm', touch_links = ['rh_ff_biotac_link', 'rh_ffdistal'])
                current_step = 5
            # Step 3.2.6
            if current_step == 5:
                rospy.loginfo('Moving backward in a straight line to try to tilt the book.')
                self.move_forward(-0.05, eef_step = 0.01)
                self.arm_and_fingers.detach_object(self.book_id)
                rospy.loginfo('Tilting finished.')
            return MoveItErrorCodes(1)
        except MoveItCommanderException as e:
            rospy.logerr(e)
            response = self.__report_error(self.tilt_book.__name__, current_step)
            if response == 'q':
                return MoveItErrorCodes(99999)
            elif response == 'r':
                return self.tilt_book(lift, start_step = current_step)
            elif response == 'c':
                return self.tilt_book(lift, start_step = current_step + 1)
            elif response.isnumeric():
                return self.tilt_book(lift, start_step = int(response))

    # Interface part of step 3.4.1 and 3.4.2
    def move_from_book(self, start_step = 0):
        '''
        Moves the shadow hand away from the book it has just tilted.

        :param int32 start_step: The step of this process which should be used
            as the starting step.

        :return MoveItErrorCodes: Corresponding error code to the success or
            failure of this process.
        '''
        current_step = start_step
        try:
            # Step 3.4.1
            if current_step == 0:
                self.move_upward(0.03)
                current_step = 1
            # Step 3.4.2
            if current_step == 1:
                current_pose = self.arm_and_hand.get_current_pose(end_effector_link = 'rh_ff_biotac_link')
                position = current_pose.pose.position
                position.x += -0.1
                position.y += -0.15
                self.move_position_bio_ik(position, direction = False, lift = None)
            return MoveItErrorCodes(1)
        except MoveItCommanderException as e:
            rospy.logerr(e)
            response = self.__report_error(self.tilt_book.__name__, current_step)
            if response == 'q':
                return MoveItErrorCodes(99999)
            elif response == 'r':
                return self.move_from_book(start_step = current_step)
            elif response == 'c':
                return self.move_from_book(start_step = current_step + 1)
            elif response.isnumeric():
                return self.move_from_book(start_step = int(response))

    def move_position_bio_ik(self, position, direction = False, lift = 0.87):
        '''
        Moves the gripper to the specified position, using first a bio ik
        generated pose and then OMPL planning.

        :param Point position: The position given to the IK solver.
        :param bool direction: Whether or not a direction goal should be added
            to the IK solver.
        :param float64 lift: An additional constraint for the right shoulder
            lift for the bio ik solver.
        '''
        debug_bio_ik_pub = rospy.Publisher('~debug_sh_bio_ik', DisplayRobotState, queue_size = 1, latch = True)
        rospy.wait_for_service('/bio_ik/get_bio_ik')
        bio_ik_service = rospy.ServiceProxy('/bio_ik/get_bio_ik', GetIK)
        request = self.prepare_request(5)
        request = self.add_goals(request, position, direction, lift)
        response = bio_ik_service(request).ik_response
        if not response.error_code.val == 1:
            debug_robot_state = DisplayRobotState()
            debug_robot_state.state.joint_state = response.solution.joint_state
            debug_bio_ik_pub.publish(debug_robot_state)
            raise MoveItCommanderException("Bio_ik planning failed with error code {}.".format(response.error_code.val))
        filtered_joint_state = self.filter_joint_state(response.solution.joint_state, self.arm_and_hand)
        self.arm_and_hand.set_joint_value_target(filtered_joint_state)
        plan = self.arm_and_hand.go(wait = True)
        if not plan:
            raise MoveItCommanderException("Moving to the initial pose failed. No path was found to the joint state \n {}.".format(filtered_joint_state))
        self.arm_and_hand.stop()

    def prepare_request(self, timeout_seconds):
        '''
        Prepares an IKRequest for the bio ik solver.

        :param float64 timeout_seconds: Number of seconds after which the solver
            stops if no pose was found.

        :return IKRequest: The prepared IKRequest.
        '''
        request = IKRequest()
        request.group_name = 'right_arm'
        request.approximate = True
        request.timeout = rospy.Duration.from_sec(timeout_seconds)
        request.avoid_collisions = True
        request.robot_state = self.robot.get_current_state()
        return request

    def add_goals(self, request, position, direction, lift):
        '''
        Adds the necessary goals to the provided IKRequest with the provided
        parameters.

        :param IKRequest request: The IKRequest to which the goals should be
            added to.
        :param Point position: The position that should be added as a goal
            to the IKRequest.
        :param bool direction: Whether or not a direction goal should be added
            to the IKRequest.
        :param float64 lift: The constraint value for the r_shoulder_lift_goal
            joint. Set to None is not desired.

        :return IKRequest: The IKRequest with the added goals.
        '''
        ff_tip_pos_goal = PositionGoal()
        ff_tip_pos_goal.link_name = 'rh_ff_biotac_link'
        ff_tip_pos_goal.weight = 5.0
        ff_tip_pos_goal.position = position
        request.position_goals = [ff_tip_pos_goal]

        # Change following values to counter calibration errors
        if not self.demo:
            ff_tip_pos_goal.position.x += 0
            ff_tip_pos_goal.position.y += 0
            ff_tip_pos_goal.position.z += 0

        if self.visualize:
            test_pose = PoseStamped()
            test_pose.header.frame_id = self.base_frame
            test_pose.pose.position = ff_tip_pos_goal.position
            self.debug_pub.publish(test_pose)

        if direction:
            debug_dir_pub = rospy.Publisher('~debug_sh_direction', Marker, queue_size = 1, latch = True)
            debug_marker = Marker()
            debug_marker.header.frame_id = 'base_footprint'
            debug_marker.ns = 'direction'
            debug_marker.id = 0
            debug_marker.type = 0
            debug_marker.action = 0
            debug_marker.lifetime = rospy.Duration(0)
            debug_marker.pose.orientation.w = 1
            debug_marker.color.a = 1.0
            debug_marker.color.r = 1.0
            debug_marker.scale = Vector3(0.01, 0.01, 0.01)
            ff_tip_dir_goal = DirectionGoal()
            ff_tip_dir_goal.link_name = 'rh_ff_biotac_link'
            ff_tip_dir_goal.weight = 2.0
            if self.finger_flat:
                ff_tip_dir_goal.axis = Vector3(0, 0, 1)
                ff_tip_dir_goal.direction = Vector3(0.3, 0.2, 0.5)
            else:
                ff_tip_dir_goal.axis = Vector3(1, 0, 0)
                ff_tip_dir_goal.direction = Vector3(0, 0, -1)
            request.direction_goals = [ff_tip_dir_goal]
            end_point = deepcopy(position)
            end_point.x += ff_tip_dir_goal.direction.x
            end_point.y += ff_tip_dir_goal.direction.y
            end_point.z += ff_tip_dir_goal.direction.z
            debug_marker.points = [position, end_point]
            debug_dir_pub.publish(debug_marker)

        if not lift is None:
            r_shoulder_lift_goal = JointVariableGoal()
            r_shoulder_lift_goal.variable_name = 'r_shoulder_lift_joint'
            r_shoulder_lift_goal.variable_position = lift
            r_shoulder_lift_goal.weight = 0.7
            r_shoulder_lift_goal.secondary = True
            request.joint_variable_goals = [r_shoulder_lift_goal]

        return request

    def filter_joint_state(self, joint_state, move_group):
        '''
        Filters the given joint state for only the joint specified in the given
        move group.

        :param JointState joint_state: The joint state to be filtered.
        :param MoveGroup move_group: The move group used as the filter.

        :return JointState: The filtered joint state.
        '''
        filtered_joint_state = joint_state
        joint_names = move_group.get_active_joints()
        indices = [filtered_joint_state.name.index(joint_name) for joint_name in joint_names]
        filtered_joint_state.name = joint_names
        filtered_joint_state.position = [joint_state.position[x] for x in indices]
        if joint_state.velocity:
            filtered_joint_state.velocity = [joint_state.velocity[x] for x in indices]
        if joint_state.effort:
            filtered_joint_state.effort = [joint_state.effort[x] for x in indices]
        return filtered_joint_state

    def move_ff_hook(self):
        '''
        Moves the first finger of the right hand into a hooked position for
        tilting the book.
        '''
        self.fingers.set_joint_value_target('rh_FFJ2', 1.484)
        plan = self.fingers.go(wait = True)
        if not plan:
            raise MoveItCommanderException("Moving the first finger into hook posistion failed.")
        self.fingers.stop()

    def move_ff_flat(self):
        '''
        Moves the first finger of the right hand into a flat position and the
        other fingers into a folded position for tilting the book.
        '''
        self.fingers.set_joint_value_target('rh_FFJ2', 0)
        self.fingers.set_joint_value_target('rh_FFJ3', 0)
        self.fingers.set_joint_value_target('rh_MFJ2', 1.57)
        self.fingers.set_joint_value_target('rh_MFJ3', 1.57)
        self.fingers.set_joint_value_target('rh_RFJ2', 1.57)
        self.fingers.set_joint_value_target('rh_RFJ3', 1.57)
        self.fingers.set_joint_value_target('rh_LFJ2', 0)
        self.fingers.set_joint_value_target('rh_LFJ3', 1.57)
        plan = self.fingers.go(wait = True)
        if not plan:
            raise MoveItCommanderException("Moving the fingers into the pick position failed.")
        self.fingers.stop()

    def move_torso_shelf(self, shelf_nr):
        '''
        Moves the robots torso to a height corresponding to the specified shelf.

        :param int32 shelf_nr: The number of the shelf the robots torso should
            be moved to respectively.
        '''
        if shelf_nr == 0:
            height = 0.168
        elif shelf_nr == 1:
            height = 0.168 + 0.132 # base height + offset for shelf difference
        else:
            raise MoveItCommanderException("{} is not a known shelf number. Please try again with a valid number.".format(shelf_nr))
        self.torso.set_joint_value_target([height])
        plan = self.torso.go(wait = True)
        if not plan:
            raise MoveItCommanderException("Moving the torso to shelf {} at height {} failed.".format(shelf_nr, height))

    def move_upward(self, length, tolerance = 0.9):
        '''
        Moves the robot arm along the z-axis relative to the
        base_frame for the specified length.

        :param float64 length: The length to move in m.
        :param float64 tolerance: A value in [0,1] which specifies the required
            percentage of the the path being found.
        '''
        pose = self.arm_and_hand.get_current_pose()
        pose.header.frame_id = self.base_frame
        pose.pose.position.z += length
        (plan, fraction) = self.arm_and_hand.compute_cartesian_path([pose.pose], 0.01, 5.0)
        if (fraction > tolerance):
            self.arm_and_hand.execute(plan)
        else:
            raise MoveItCommanderException("Moving upward failed. No cartesian path found to move {} along the z-axis of the base frame with a tolerance of {}. Only {} of the path was found.".format(length, tolerance, fraction))
        rospy.sleep(1)

    def move_forward(self, length, tolerance = 0.9, eef_step = 0.01):
        '''
        Moves the robot arm along the x-axis relative to the
        base_frame for the specified length.

        :param float64 length: The length to move in m.
        :param float64 tolerance: A value in [0,1] which specifies the required
            percentage of the the path being found.
        :param float64 eef_step: The eef_step parameter for the
            compute_cartesian_path function.
        '''
        pose = self.arm_and_hand.get_current_pose()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x += length
        (plan, fraction) = self.arm_and_hand.compute_cartesian_path([pose.pose], eef_step, 0.0, avoid_collisions = True)
        if (fraction > tolerance):
            self.arm_and_hand.execute(plan)
        else:
            raise MoveItCommanderException("Moving forward failed. No cartesian path found to move {} along the x-axis of the base frame with a tolerance of {}. Only {} of the path was found.".format(length, tolerance, fraction))
        rospy.sleep(1)

    def move_right(self, length, tolerance = 0.9):
        '''
        Moves the robot arm along the y-axis relative to the
        base_frame for the specified length.

        :param float64 length: The length to move in m.
        :param float64 tolerance: A value in [0,1] which specifies the required
            percentage of the the path being found.
        '''
        pose = self.arm_and_hand.get_current_pose()
        pose.header.frame_id = self.base_frame
        pose.pose.position.y += length
        (plan, fraction) = self.arm_and_hand.compute_cartesian_path([pose.pose], 0.01, 5.0)
        if (fraction > tolerance):
            self.arm_and_hand.execute(plan)
        else:
            raise MoveItCommanderException("Moving forward failed. No cartesian path found to move {} along the y-axis of the base frame with a tolerance of {}. Only {} of the path was found.".format(length, tolerance, fraction))
        rospy.sleep(1)

    def open_fingers(self):
        '''
        Opens all fingers of the right hand.
        '''
        try:
            self.set_joints_current(self.fingers)
            self.fingers.set_joint_value_target('rh_FFJ4', 0)
            self.fingers.set_joint_value_target('rh_FFJ3', 0)
            self.fingers.set_joint_value_target('rh_FFJ2', 0.0175)
            self.fingers.set_joint_value_target('rh_MFJ4', 0)
            self.fingers.set_joint_value_target('rh_MFJ3', 0)
            self.fingers.set_joint_value_target('rh_MFJ2', 0.0175)
            self.fingers.set_joint_value_target('rh_RFJ4', 0)
            self.fingers.set_joint_value_target('rh_RFJ3', 0)
            self.fingers.set_joint_value_target('rh_RFJ2', 0.0175)
            self.fingers.set_joint_value_target('rh_LFJ5', 0.0175)
            self.fingers.set_joint_value_target('rh_LFJ4', 0)
            self.fingers.set_joint_value_target('rh_THJ5', 0)
            self.fingers.set_joint_value_target('rh_THJ4', 0.0175)
            self.fingers.set_joint_value_target('rh_THJ3', 0)
            self.fingers.set_joint_value_target('rh_THJ2', 0)
            self.fingers.go(wait = True)
            self.fingers.stop()
        except MoveItCommanderException:
            raise

    def set_joints_current(self, move_group):
        '''
        Sets all joints in the specified move group to the current values.

        :param MoveGroup move_group: The move group containing the joints.
        '''
        current_joint_values = move_group.get_current_joint_values()
        move_group.set_joint_value_target(current_joint_values)

    def get_book_dimensions(self, halved = False):
        '''
        Provides the dimensions of the book object.

        :param bool halved: Set if dimensions should be returned halved or full.

        :return list: List of the books dimensions as float64.
        '''
        book = self.scene.get_objects([self.book_id])[self.book_id]
        book_dimensions = book.primitives[0].dimensions
        if halved:
            depth = book_dimensions[0]/2
            width = book_dimensions[1]/2
            height = book_dimensions[2]/2
            book_dimensions = (depth, width, height)
        return book_dimensions

    def get_book_pose(self):
        '''
        Provides the current pose of the not-attached book object relative to
        the base_frame.

        :return PoseStamped: Pose of the book object.
        '''
        book_pose = PoseStamped()
        book_pose.header.frame_id = self.base_frame
        book_pose.pose = self.scene.get_object_poses([self.book_id])[self.book_id]
        return book_pose

    def attach_book(self):
        '''
        Attaches the book to the right hand and allows the right fingers to
        collide with the book.
        '''
        self.fingers.attach_object(self.book_id, touch_links = ['rh_thtip','rh_th_biotac_link','rh_thdistal','rh_fftip','rh_ff_biotac_link','rh_ffdistal','rh_mftip','rh_mf_biotac_link','rh_mfdistal','rh_rftip','rh_rf_biotack_link','rh_rfdistal','rh_lftip','rh_lf_biotac_link','rh_lfdistal'])

    def rotate_quaternion(self, euler_angles, old_quaternion):
        '''
        Rotates the given quaternion by the specified euler angles.

        :param tuple euler_angles: A tuple of three float64 by which the
            quaternion should br rotated, specified in rad.
        :param Quaternion old_quaternion: The quaternion that should be rotated.

        :return Quaternion: The rotated quaternion.
        '''
        change_quaternion = quaternion_from_euler(*euler_angles)
        old_quaternion = [old_quaternion.x, old_quaternion.y, old_quaternion.z, old_quaternion.w]
        new_quaternion = Quaternion(*quaternion_multiply(change_quaternion, old_quaternion).tolist())
        return new_quaternion

    def __report_error(self, func_name, step):
        '''
        Helper function to properly handle occuring errors by asking the user
        how to proceed.
        '''
        response = input('An error has occured in function {} in step {}. Do you wish to quit execution (q), retry the step (r) or continue without the step (c) ?'
                         ' Alternatively, enter a number to specify from which step to redo the current function.\n'.format(func_name, step))
        while not (response in {'q', 'r', 'c'} or response.isnumeric()):
            response = input('Unknown input received. Please enter again if you want to quit (q), retry (r), continue (c) or restart from a specific step (number).\n')
        return response

    # DEPRECATED. Not currently used in the pipeline, but can be used manually.
    def setup(self, start_step = 0):
        '''
        Prepares the right arm in a save position for the book tilting. This
        includes moveing the arm to the right side of the robot and resetting
        the fingers and wrist.

        :param int32 start_step: The step of this process which should be used
            as the starting step.

        :return MoveItErrorCodes: Corresponding error code to the success or
            failure of this process.
        '''
        current_step = start_step
        try:
            if current_step == 0:
                rospy.loginfo('Moving the arm to the side of the robot into a save position.')
                self.arm.set_named_target('right_arm_to_side')
                plan = self.arm.go(wait = True)
                self.arm.stop()
                current_step = 1
            if current_step == 1:
                rospy.loginfo('Opening the fingers.')
                self.open_fingers()
                current_step = 2
            if current_step == 2:
                rospy.loginfo('Resetting the wrist tilt to 0.')
                self.set_joints_current(self.arm_and_hand)
                self.arm_and_hand.set_joint_value_target('rh_WRJ1', 0)
                self.arm_and_hand.go(wait = True)
                self.arm_and_hand.stop()
            return MoveItErrorCodes(1)
        except MoveItCommanderException as e:
            rospy.logerr(e)
            response = self.__report_error(self.setup.__name__, current_step)
            if response == 'q':
                return MoveItErrorCodes(99999)
            elif response == 'r':
                return self.setup(start_step = current_step)
            elif response == 'c':
                return self.setup(start_step = current_step + 1)
            elif response.isnumeric():
                return self.setup(start_step = int(response))

