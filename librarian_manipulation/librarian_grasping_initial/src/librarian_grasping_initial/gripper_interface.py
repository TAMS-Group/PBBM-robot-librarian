#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped, Quaternion
from moveit_msgs.msg import MoveItErrorCodes, CollisionObject
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction
from math import cos, sin, fabs
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
from actionlib import SimpleActionClient
from bio_ik_msgs.msg import PoseGoal, JointVariableGoal, IKRequest
from bio_ik_msgs.srv import GetIK
from std_msgs.msg import String
from moveit_commander.exception import MoveItCommanderException
from tf.transformations import quaternion_multiply, quaternion_from_euler
from copy import deepcopy

class GripperInterface():

    def __init__(self, general_controllers, arm_controllers, book_id = 'book', demo = False):
        self.robot = general_controllers[0]
        self.scene = general_controllers[1]
        self.torso = general_controllers[2]
        self.torso.set_max_velocity_scaling_factor(0.9)
        self.torso.set_max_acceleration_scaling_factor(0.9)
        self.arm = arm_controllers[0]
        self.gripper = arm_controllers[1]
        self.gripper.set_max_velocity_scaling_factor(0.9)
        self.gripper.set_max_acceleration_scaling_factor(0.9)
        self.base_frame = 'base_footprint'
        self.book_id = book_id
        self.demo = demo
        try:
            self.book_dimensions = self.get_book_dimensions()
        except MoveItCommanderException as e:
            rospy.logerr(e)
        self.debug_pub = rospy.Publisher('debug_gripper_interface', PoseStamped, queue_size = 1)

    # Interface part of step 3.3
    def grasp_book(self, grasp_pose, start_step = 0, lift = 0.52):
        '''
        Grasps the book at the specified position and moves it in front of the
        shelf. The position specified needs to be 0.13 on the x-axis of the base
        frame before the desired grasp point.

        :param PoseStamped grasp_pose: Position of the eef for grasping the book
        :param int32 start_step: The step of this process which should be used
            as the starting step.
        :param lift: A constraint value for the left shoulder lift given to the
            bio ik solver for moving to the grasp pose.

        :return MoveItErrorCodes: Corresponding error code to the success or
            failure of this process.
        '''
        current_step = start_step
        try:
            # Step 3.3.1
            if current_step == 0:
                rospy.loginfo('Moving to initial grasping pose')
                self.move_to_pose_bio_ik(grasp_pose, lift)
                current_step = 1
            # Step 3.3.2
            if current_step == 1:
                rospy.loginfo('Moving forward.')
                self.move_forward(0.075, tolerance = 0.9)
                current_step = 2
            # Step 3.3.3
            if current_step == 2:
                rospy.loginfo('Attaching book and closing gripper.')
                self.attach_book()
                self.close_gripper()
            return MoveItErrorCodes(1)
        except MoveItCommanderException as e:
            rospy.logerr(e)
            response = self.__report_error(self.grasp_book.__name__, current_step)
            if response == 'q':
                return MoveItErrorCodes(99999)
            elif response == 'r':
                return self.grasp_book(grasp_pose, start_step = current_step, lift = lift)
            elif response == 'c':
                return self.grasp_book(grasp_pose, start_step = current_step + 1, lift = lift)
            elif response.isnumeric():
                return self.grasp_book(grasp_pose, start_step = int(response), lift = lift)

    # Interface part of step 3.4.2 - 3.4.7
    def extract_book_regrasp(self, start_step = 0):
        '''
        Extract the book this interface was initialized with, using the regrasp
        methods, which involves letting the book fall back during the process
        for a more stable grasp.

        :param int32 start_step: The step of this process which should be used
            as the starting step.

        :return MoveItErrorCodes: Corresponding error code to the success or
            failure of this process.
        '''
        current_step = start_step
        try:
            acm = deepcopy(self.scene.get_collision_matrix())
            self.scene.disable_collision_detections(self.book_id, self.scene.get_known_object_names())
            # Step 3.4.2
            if current_step == 0:
                rospy.loginfo('Moving backwards.')
                self.move_forward(-0.04)
                current_step = 1
            # Step 3.4.3
            if current_step == 1:
                rospy.loginfo('Opening gripper and detaching book.')
                self.open_gripper()
                self.detach_book()
                current_step = 2
            # Step 3.4.4
            if current_step == 2:
                rospy.loginfo('Moving downward.')
                self.move_upward(-0.05)
                current_step = 3
            # Step 3.4.5
            if current_step == 3:
                rospy.loginfo('Moving gripper forward.')
                self.attach_book() # attaching and detaching here is just a workaround for not being able to move the book itself yet
                self.move_forward(0.04)
                self.detach_book()
                current_step = 4
            # Step 3.4.6
            if current_step == 4:
                rospy.loginfo('Attaching book and closing gripper.')
                self.attach_book()
                self.close_gripper()
                current_step = 5
            # Step 3.4.7
            if current_step == 5:
                rospy.loginfo('Moving gripper backward.')
                self.move_forward(-self.book_dimensions[0] - 0.02, tolerance = 0.5)
            self.scene.set_collision_matrix(acm)
            rospy.loginfo('Grasping process finished successful.')
            return MoveItErrorCodes(1)
        except MoveItCommanderException as e:
            rospy.logerr(e)
            response = self.__report_error(self.extract_book_regrasp.__name__, current_step)
            if response == 'q':
                return MoveItErrorCodes(99999)
            elif response == 'r':
                return self.extract_book_regrasp(start_step = current_step)
            elif response == 'c':
                return self.extract_book_regrasp(start_step = current_step + 1)
            elif response.isnumeric():
                return self.extract_book_regrasp(start_step = int(response))

    # Interface part of step 5.2
    def place_book_vertical(self, adjacent_object, direction, start_step = 0):
        '''
        Places the book in the gripper vertical, but a bit tilted, against the
        specified object. This object can be either specified as a
        CollisionObject or a PoseStamped.

        :param PoseStamped/CollisionObject adjacent_object: If specified as a
            CollisionObject, it needs to be the collision object the book should
            be placed against. If specified as a PoseStamped, it needs to be
            either the bottom right corner of the adjacent object or wall the
            book should be placed against, if the gripper should approach it
            from the right, or the bootom left corner if approached from the
            left.
        :param String direction: 'left' if the gripper should approach from the
            left, or 'right' if the gripper should approach form the right.
        :param int32 start_step: The step of this process which should be used
            as the starting step.

        :return MoveItErrorCodes: Corresponding error code to the success or
            failure of this process.
        '''
        tf_buffer = Buffer()
        listener = TransformListener(tf_buffer)
        rospy.sleep(1)

        if direction == 'left':
            direction_factor = 1
        elif direction == 'right':
            direction_factor = -1
        else:
            raise MoveItCommanderException('Unknown direction {}. Please use either left or right as direction.'.format(direction))

        if isinstance(adjacent_object, PoseStamped):
            target_pose = adjacent_object
            transform = tf_buffer.lookup_transform('base_footprint', target_pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            target_pose = do_transform_pose(target_pose, transform)
        elif isinstance (adjacent_object, CollisionObject):
            target_pose = PoseStamped()
            target_pose.pose = adjacent_object.pose
            transform = tf_buffer.lookup_transform('base_footprint', target_pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            target_pose = do_transform_pose(target_pose, transform)
            target_pose.pose.position.x += - adjacent_object.primitives[0].dimensions[0]
            target_pose.pose.position.z += - adjacent_object.primitives[0].dimensions[2]
            target_pose.pose.position.y += direction_factor * adjacent_object.primitives[0].dimensions[1]
        else:
            raise MoveItCommanderException('{} is an unsupported argument for adjacent_object. Please provide either a CollisionObject or a PoseStamped.'.format(adjacent_object))

        target_pose.pose.position.z += self.book_dimensions[2]/2 - 0.01 # Half of book height - offset
        target_pose.pose.position.y += direction_factor * (self.book_dimensions[1]/2 + 0.04 + 0.05) # Half of book width + width of tilted book + offset
        target_pose.pose.position.x += - self.book_dimensions[0] - 0.05 # - Book depth - offset
        target_pose.pose.orientation.x = 0
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 1
        target_pose.pose.orientation = self.rotate_quaternion((direction_factor * 0.38, 0, 0), target_pose.pose.orientation)

        self.debug_pub.publish(target_pose)

        current_step = start_step
        try:
            # Step 5.2.1
            if current_step == 0:
                rospy.loginfo('Moving to initial place pose.')
                self.move_to_pose_bio_ik(target_pose)
                current_step = 1
            # Step 5.2.2
            if current_step == 1:
                rospy.loginfo('Moving forward.')
                self.move_forward(self.book_dimensions[0] + 0.1)
                current_step = 2
            # Step 5.2.3
            if current_step == 2:
                rospy.loginfo('Moving sidewards.')
                if self.demo:
                    self.move_right(-direction_factor * 0.07, tolerance = 0.01)
                else:
                    self.move_right_force(direction = 'right')
                current_step = 3
            # Step 5.2.4
            if current_step == 3:
                rospy.loginfo('Moving downwards.')
                if self.demo:
                    self.move_upward(-0.06, tolerance = 0.01)
                else:
                    self.move_upward_force()
                current_step = 4
            # Step 5.2.5
            if current_step == 4:
                rospy.loginfo('Opening gripper and detaching book.')
                rospy.loginfo('Opening gripper for a book with width {}.'.format(self.book_dimensions[1]))
                self.open_gripper_partial(self.book_dimensions[1] + 0.01) # open book width + additional offset to completely remove gripper from book
                self.detach_book()
                rospy.sleep(1)
                current_step = 5
            # Step 5.2.6
            if current_step == 5:
                rospy.loginfo('Moving backwards.')
                self.move_forward(- 0.12)
                current_step = 6
            # Step 5.2.7
            if current_step == 6:
                rospy.loginfo('Close gripper.')
                self.close_gripper()
                rospy.loginfo('Placing successful.')
            return MoveItErrorCodes(1)
        except MoveItCommanderException as e:
            rospy.logerr(e)
            response = self.__report_error(self.place_book_vertical.__name__, current_step)
            if response == 'q':
                return MoveItErrorCodes(99999)
            elif response == 'r':
                return self.place_book_vertical(adjacent_object, direction, start_step = current_step)
            elif response == 'c':
                return self.place_book_vertical(adjacent_object, direction, start_step = current_step + 1)
            elif response.isnumeric():
                return self.place_book_vertical(adjacent_object, direction, start_step = int(response))

    # Interface part of step 5.3
    def push_book_upright(self, direction, start_step = 0):
        '''
        Pushes the book, which leans against a wall or object, upright.

        :param String direction: 'left' if the gripper should approach from the
            left, or 'right' if the gripper should approach form the right.
        :param int32 start_step: The step of this process which should be used
            as the starting step.

        :return MoveItErrorCodes: Corresponding error code to the success or
            failure of this process.
        '''
        rospy.loginfo('Begin to push book upright.')

        if direction == 'left':
            direction_factor = 1
        elif direction == 'right':
            direction_factor = -1
        else:
            raise MoveItCommanderException('Unknown direction {}. Please use either left or right as direction.'.format(direction))

        current_step = start_step
        try:
            book_dimensions_halved = self.get_book_dimensions(halved = True)
            # Step 5.3.1
            if current_step == 0:
                rospy.loginfo('Moving next to the book.')
                target_pose = self.get_grasp_pose(distance = 0.05)
                target_pose.pose.position.y += direction_factor * (book_dimensions_halved[1] + 0.07) # Gripper width + safety margin
                target_pose.pose.orientation = Quaternion(*quaternion_from_euler(1.5708, 0, 0).tolist())
                self.move_to_pose(target_pose)
                current_step = 1
            # Step 5.3.2
            if current_step == 1:
                rospy.loginfo('Moving forward.')
                self.move_forward(book_dimensions_halved[0] + 0.09)
                current_step = 2
            # Step 5.3.3
            if current_step == 2:
                rospy.loginfo('Pushing_book_sidewards.')
                self.rotate_book(- direction_factor * 0.38)
                self.attach_book()
                if self.demo:
                    self.move_right(- direction_factor * 0.05, 0.01)
                else:
                    self.move_right_force(direction = direction, threshold = 4, force_direction_x = False)
                self.detach_book()
                current_step = 3
            # Step 5.3.4
            if current_step == 3:
                rospy.loginfo('Moving sidewards.')
                self.move_right(direction_factor * 0.05, tolerance = 0.01)
                current_step = 4
            # Step 5.3.5
            if current_step == 4:
                rospy.loginfo('Moving backwards.')
                self.move_forward(-0.16)
                current_step = 5
            # Step 5.3.6
            if current_step == 5:
                rospy.loginfo('Opening gripper.')
                self.open_gripper()
                rospy.loginfo('Pushing book upright finished.')
            return MoveItErrorCodes(1)
        except MoveItCommanderException as e:
            rospy.logerr(e)
            response = self.__report_error(self.push_book_upright.__name__, current_step)
            if response == 'q':
                return MoveItErrorCodes(99999)
            elif response == 'r':
                return self.push_book_upright(direction, start_step = current_step)
            elif response == 'c':
                return self.push_book_upright(direction, start_step = current_step + 1)
            elif response.isnumeric():
                return self.push_book_upright(direction, start_step = int(response))

    def get_grasp_pose(self, distance = 0.10, height = 0):
        '''
        Provides the initial pose for grasping the book, depending on the
        given parameters.

        :param float64 distance: The distance the gripper should be in front of
            the book spine.
        :param float64 height: The offset along the book spine for the grasping
            pose relative to the spine center.

        :return PoseStamped: The pose of the eef to grasp the book.
        '''
        grasp_pose = self.get_book_pose()
        book_dimensions_halved = self.get_book_dimensions(True)
        grasp_pose.pose.position.x += -book_dimensions_halved[0] - distance # - half of book depth - offset
        grasp_pose.pose.position.z += height
        return grasp_pose

    def get_book_pose(self):
        '''
        Provides the current pose of the not-attached book object relative to
        the base_frame.

        :return PoseStamped: Pose of the book object.
        '''
        book_pose = PoseStamped()
        book_pose.header.frame_id = self.base_frame
        result = {}
        while True:
            result = self.scene.get_object_poses([self.book_id])
            if self.book_id in result:
                break
            rospy.logerr('Object with id {} was not found in the planning scene.'.format(self.book_id))
        book_pose.pose = result[self.book_id]
        return book_pose

    def get_book_dimensions(self, halved = False):
        '''
        Provides the dimensions of the book object.

        :param bool halved: Set if dimensions should be returned halved or full.

        :return list: List of the books dimensions as float64.
        '''
        result = {}
        while True:
            result = self.scene.get_objects([self.book_id])
            if self.book_id in result:
                break
            rospy.logerr('Object with id {} was not found in the planning scene.'.format(self.book_id))
        book = result[self.book_id]
        book_dimensions = book.primitives[0].dimensions
        if halved:
            depth = book_dimensions[0]/2
            width = book_dimensions[1]/2
            height = book_dimensions[2]/2
            book_dimensions = (depth, width, height)
        return book_dimensions

    def move_to_pose(self, pose):
        '''
        Moves the gripper to the specified pose using simple OMPL planning.

        :param PoseStamped pose: The pose for the gripper.
        '''
        self.arm.set_pose_target(pose)
        plan = self.arm.go(wait=True)
        if not plan:
            raise MoveItCommanderException("Moving to pose failed. No path found to pose\n {} \n Aborting movement".format(pose))
        self.arm.stop()

    def move_to_pose_bio_ik(self, pose, lift = 0.69):
        '''
        Moves the gripper to the specified pose using a bio ik generated pose
        and OMPL planning.

        :param PoseStamped pose: The pose for the gripper.
        :param float64 lift: An additional constraint for the left shoulder lift
            for the bio ik solver.
        '''
        rospy.wait_for_service('/bio_ik/get_bio_ik')
        bio_ik_service = rospy.ServiceProxy('/bio_ik/get_bio_ik', GetIK)
        request = self.prepare_request(5)
        request = self.add_goals(request, pose.pose, lift)
        response = bio_ik_service(request).ik_response
        if not response.error_code.val == 1:
            raise MoveItCommanderException("Bio_ik planning failed with error code {}.".format(response.error_code.val))
        filtered_joint_state = self.filter_joint_state(response.solution.joint_state, self.arm)
        self.arm.set_joint_value_target(filtered_joint_state)
        plan = self.arm.go(wait = True)
        if not plan:
            raise MoveItCommanderException("Moving to pose \n {} \n failed. No path was found to the joint state \n {}.".format(pose, filtered_joint_state))
        self.arm.stop()

    def prepare_request(self, timeout_seconds = 1):
        '''
        Prepares an IKRequest for the bio ik solver.

        :param float64 timeout_seconds: Number of seconds after which the solver
            stops if no pose was found.

        :return IKRequest: The prepared IKRequest.
        '''
        request = IKRequest()
        request.group_name = 'left_arm'
        request.approximate = True
        request.timeout = rospy.Duration.from_sec(timeout_seconds)
        request.avoid_collisions = True
        request.robot_state = self.robot.get_current_state()
        return request

    def add_goals(self, request, pose, lift):
        '''
        Adds the necessary goals to the provided IKRequest.

        :param IKRequest request: The IKRequest to which the goals should be
            added to.
        :param PoseStamped pose: The pose goal for the gripper.
        :param float64 lift: The constraint value for the l_shoulder_lift_goal
            joint. Set to None if not desried.

        :return IKRequest: The IKRequest with the added goals.
        '''
        eef_pose_goal = PoseGoal()
        eef_pose_goal.link_name = 'l_gripper_tool_frame'
        eef_pose_goal.weight = 10.0
        eef_pose_goal.pose = pose
        eef_pose_goal.rotation_scale = 0.0
        request.pose_goals = [eef_pose_goal]

        if not lift is None:
            l_shoulder_lift_goal = JointVariableGoal()
            l_shoulder_lift_goal.variable_name = 'l_shoulder_lift_joint'
            l_shoulder_lift_goal.variable_position = lift
            l_shoulder_lift_goal.weight =2.0
            l_shoulder_lift_goal.secondary = False
            request.joint_variable_goals = [l_shoulder_lift_goal]

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

    def move_home(self):
        '''
        Moves the robot arm back into its default location on the left side of
        the robot.
        '''
        self.arm.set_named_target('left_arm_to_side')
        plan = self.arm.go(wait=True)
        if not plan:
            raise MoveItCommanderException("Moving home failed. No path found to pose\n {} \n Aborting movement".format(pose))
        self.arm.stop()

    def move_book_camera(self):
        '''
        Moves the gripper holding the book 30 cm in front of the robots camera
        and turns it by 180° after a short wait to see both sides of the book.
        '''
        pose = PoseStamped()
        pose.header.frame_id = 'azure_kinect_rgb_sensor'
        pose.pose.position.x = 0.3
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, -1.57))
        self.debug_pub.publish(pose)
        try:
            self.move_to_pose(pose)
            rospy.sleep(2)
        except MoveItCommanderException as e:
            raise e
        current_value = self.arm.get_current_joint_values()[6]
        self.arm.set_joint_value_target(self.arm.get_current_joint_values())
        self.arm.set_joint_value_target('l_wrist_roll_joint', current_value + 3.14)
        plan = self.arm.go(wait = True)
        if not plan:
            raise MoveItCommanderException("Rotating the wrist 180 degree failed.")
        rospy.sleep(2)

    def move_cartesian_pose(self, pose, tolerance = .5):
        '''
        Moves the endeffector to the given pose with a cartesian path.

        :param PoseStamped pose: The pose for the gripper.
        :param float64 tolerance: A value in [0,1] which specifies the required
            percentage of the path being found.
        '''
        (plan, fraction) = self.arm.compute_cartesian_path([pose.pose], 0.01, 0.0)
        if (fraction > tolerance):
            self.arm.execute(plan)
        else:
            raise MoveItCommanderException("Moving to pose {} on a cartesian path failed with a tolerance of {}. Only {} of the path was found.".format(pose, tolerance, fraction))

    def move_upward(self, length, tolerance = 0.9):
        '''
        Moves the robot arm along the z-axis relative to the
        base_frame for the specified length.

        :param float64 length: The length to move in m.
        :param float64 tolerance: A value in [0,1] which specifies the required
            percentage of the path being found.
        '''
        pose = self.arm.get_current_pose()
        pose.header.frame_id = self.base_frame
        pose.pose.position.z += length
        (plan, fraction) = self.arm.compute_cartesian_path([pose.pose], 0.01, 0.0)
        if (fraction > tolerance):
            self.arm.execute(plan)
        else:
            raise MoveItCommanderException("Moving upward failed. No cartesian path found to move {} along the z-axis of the base frame with a tolerance of {}. Only {} of the path was found.".format(length, tolerance, fraction))

    def move_upward_force(self):
        '''
        Moves the gripper in a cartesian path along the z-axis of the base frame
        until a force difference threshold has been met. Only use if not in demo
        mode.
        '''
        force_sub = rospy.Subscriber('/ft/l_gripper_motor', WrenchStamped, self.__force_callback)
        rospy.sleep(0.2)
        force_offset = self.current_force
        self.scene.disable_collision_detections('shelf', [self.book_id])
        force_threshold = 2
        while not self.current_force.y > force_offset.y + force_threshold:
            self.move_upward(-0.002)
            rospy.sleep(0.02)
        rospy.loginfo('The recorded force is above the threshold.')
        self.scene.enable_collision_detections('shelf', [self.book_id])

    def move_forward(self, length, tolerance = 0.9, eef_step = 0.01):
        '''
        Moves the robot arm along the x-axis relative to the
        base_frame for the specified length.

        :param float64 length: The length to move in m.
        :param float64 tolerance: A value in [0,1] which specifies the required
            percentage of the path being found.
        :param float64 eef_step: The allowed eef_step for the cartesian path
            planning.
        '''
        pose = self.arm.get_current_pose()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x += length
        (plan, fraction) = self.arm.compute_cartesian_path([pose.pose], eef_step, 0.0)
        if (fraction > tolerance):
            self.arm.execute(plan)
        else:
             raise MoveItCommanderException("Moving forward failed. No cartesian path found to move {} along the x-axis of the base frame with a tolerance of {}. Only {} of the path was found.".format(length, tolerance, fraction))

    def move_right(self, length, tolerance = 0.9):
        '''
        Moves the robot arm along the y-axis relative to the
        base_frame for the specified length.

        :param float64 length: The length to move in m.
        :param float64 tolerance: A value in [0,1] which specifies the required
            percentage of the path being found.
        '''
        pose = self.arm.get_current_pose()
        pose.header.frame_id = self.base_frame
        pose.pose.position.y += length
        (plan, fraction) = self.arm.compute_cartesian_path([pose.pose], 0.01, 0.0)
        if (fraction > tolerance):
            self.arm.execute(plan)
        else:
             raise MoveItCommanderException("Moving forward failed. No cartesian path found to move {} along the y-axis of the base frame with a tolerance of {}. Only {} of the path was found.".format(length, tolerance, fraction))

    def move_right_force(self, direction = 'right', threshold = 2, force_direction_x = True):
        '''
        Moves the gripper in a cartesian path along the y-axis of the base frame
        until a force difference threshold has been met. Only use if not in demo
        mode.

        :param str direction: Direction from which the gripper should approach.
            Should be either 'left' or 'right'
        :param float64 threshold: The threshold for the change in forec when the
            movement should stop.
        :param bool force_direction_x: True if the x-axis should be used for the
            force detection, False if the y-axis.
        '''
        force_sub = rospy.Subscriber('/ft/l_gripper_motor', WrenchStamped, self.__force_callback)
        rospy.sleep(0.2)
        force_offset = self.current_force
        if direction == 'left':
            direction_factor = 1
        elif direction == 'right':
            direction_factor = -1
        else:
            raise MoveItCommanderException('Unknown direction {}. Please use either left or right as direction.'.format(direction))

        self.scene.disable_collision_detections('shelf', [self.book_id])
        force_threshold = threshold
        # needs inverting depending on direction
        if force_direction_x:
            while not self.current_force.x > force_offset.x + force_threshold:
                try:
                    self.move_right(- direction_factor * 0.002)
                    rospy.sleep(0.02)
                except MoveItCommanderException as e:
                    rospy.logerr(e)
                    break
        else:
            while not self.current_force.y > force_offset.y + force_threshold:
                try:
                    self.move_right(- direction_factor * 0.002)
                    rospy.sleep(0.02)
                except MoveItCommanderException as e:
                    rospy.logerr(e)
                    break
        rospy.loginfo('The recorded force is above the threshold.')
        self.scene.enable_collision_detections('shelf', [self.book_id])
        rospy.sleep(0.2)

    def close_gripper(self):
        '''
        Closes the gripper.
        '''
        self.gripper.set_named_target('closed')
        plan = self.gripper.go(wait=True)
        if not plan:
            raise MoveItCommanderException("Closing the gripper failed.")
        self.gripper.stop()

    def open_gripper(self):
        '''
        Opens the gripper.
        '''
        self.gripper.set_named_target('open')
        plan = self.gripper.go(wait=True)
        if not plan:
            raise MoveItCommanderException("Opening the gripper failed.")
        self.gripper.stop()

    def open_gripper_partial(self, width):
        '''
        Opens the gripper partially by the specified width.

        :param float64 width: The width by which the gripper should be opened.
        '''
        if not self.demo:
            gripper_client = SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
            gripper_client.wait_for_server()
            goal = Pr2GripperCommandGoal()
            goal.command.position = width
            gripper_client.send_goal(goal)
            result = gripper_client.wait_for_result(rospy.Duration.from_sec(5.0))
            if not result:
                raise MoveItCommanderException("Opening the gripper failed.")
        elif self.demo:
            self.gripper.set_joint_value_target('l_gripper_l_finger_joint', sin((width/2)/0.082)) # 0.082 = gripper length

            rospy.loginfo("Opening gripper by {}.".format(sin((width/2)/0.082)))

            plan = self.gripper.go(wait = True)
            if not plan:
                raise MoveItCommanderException("Opening the gripper failed.")

    def rotate_clockwise(self, angle):
        '''
        Rotate the gripper by the specified angle.

        :param float64 angle: The angle to rotate the gripper by around the
            x-axis of the base frame, specified in rad.
        '''
        pose = self.arm.get_current_pose()
        pose.pose.orientation = self.rotate_quaternion((angle, 0, 0), pose.pose.orientation)
        self.arm.set_pose_target(pose)
        plan = self.arm.go(wait = True)
        if not plan:
            raise MoveItCommanderException("Rotating around {} failed. No path found to pose\n {} \n".format(angle, pose))
        self.arm.stop()

    def rotate_quaternion(self, euler_angles, old_quaternion):
        '''
        Rotates the given quaternion by the specified euler angles.

        :param tuple euler_angles: A tuple of three float64 by which the
            quaternion should be rotated, specified in rad.
        :param Quaternion old_quaternion: The quaternion that should be rotated.

        :return Quaternion: The rotated quaternion.
        '''
        change_quaternion = quaternion_from_euler(*euler_angles)
        old_quaternion = [old_quaternion.x, old_quaternion.y, old_quaternion.z, old_quaternion.w]
        new_quaternion = Quaternion(*quaternion_multiply(change_quaternion, old_quaternion).tolist())
        return new_quaternion

    def rotate_book(self, angle):
        '''
        Rotate the book collision object by the specified angle.

        :param float64 angle: The angle by which the book should be rotated
            around the x-axis of the base frame, specified in rad.
        '''
        current_pose = self.get_book_pose()
        new_pose = current_pose
        new_pose.pose.orientation = self.rotate_quaternion((angle, 0, 0), current_pose.pose.orientation)
        angle = fabs(angle)
        new_pose.pose.position.z += ((self.book_dimensions[2]/2) - cos(angle) * (self.book_dimensions[2]/2))
        self.scene.add_box(self.book_id, new_pose, self.book_dimensions)

    def attach_book(self):
        '''
        Attaches the book to the gripper and allowes the gripper to collide with
        the book.
        '''
        self.gripper.attach_object(self.book_id, touch_links = ['l_gripper_l_finger_link',
                                               'l_gripper_l_finger_tip_link',
                                               'l_gripper_r_finger_link',
                                               'l_gripper_r_finger_tip_link'])

    def detach_book(self):
        '''
        Detaches the book from the gripper.
        '''
        self.gripper.detach_object(self.book_id)

    def __force_callback(self, force_readings):
        '''
        Helper functions for the force dependent move functions.
        '''
        self.current_force = force_readings.wrench.force

    def __report_error(self, func_name, step):
        '''
        Helper function to properly handle occuring errors by asking the user
        how to proceed.
        '''
        response = input('An error has occured in function {} in step {}. Do you wish to quit execution (q), retry the step (r) or continue without the step (c) ?'
                         ' Alternatively, enter a number to specify from which step to redo the current function. \n'.format(func_name, step))
        while not (response in {'q', 'r', 'c'} or response.isnumeric()):
            response = input('Unknown input received. Please enter again if you want to quit (q), retry (r), continue (c) or restart from a specific step (number).\n')
        return response

    # DEPRECATED. Not in use in the pipeline, but can still be used manually.
    def setup(self, start_step = 0):
        '''
        Setup the robot to be ready for grasping, which includes testing for the
        books existence, moving the left arm to the side of the robot and
        opening the gripper.

        :param int32 start_step: The step of this process which should be used
            as the starting step.

        :return MoveItErrorCodes: Corresponding error code to the success or
            failure of this process.
        '''
        current_step = start_step
        try:
            if current_step == 0:
                rospy.loginfo('Testing for the existence of the book.')
                self.test_book()
                current_step = 1
            if current_step == 1:
                rospy.loginfo('Moving home.')
                self.move_home()
                current_step = 2
            if current_step == 2:
                rospy.loginfo('Opening gripper.')
                self.open_gripper()
                rospy.loginfo('Setup finished succesfully.')
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

    def test_book(self):
        '''
        Tests if book is a valid collision object in the planning scene. Raises
        an exception if not.
        '''
        if not self.book_id in self.scene.get_objects():
            raise MoveItCommanderException("The object book currently does not exist in the planning scene. Further execution is aborted. Please try again after the object was created.")

