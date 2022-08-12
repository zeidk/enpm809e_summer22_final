#!/usr/bin/env python3

import rospy
import sys
import tf

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sqrt, atan2, pi
import tf2_ros
from std_srvs.srv import Empty
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel
from enpm809e_msgs.msg import PartInfo, PartInfos, LogicalCameraImage, Model
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion, Pose, TransformStamped, Twist, Point
from bot_controller.part_class import Part


class BotController(object):
    """
    A controller class to drive a turtlebot in Gazebo.
    """

    def __init__(self, rate=10):
        rospy.init_node('bot_controller')
        rospy.loginfo('Press Ctrl c to exit')
        self._is_part_info_gathered = False
        self._part_list = []
        self._camera_parts = []

        #  used to check whether a camera data has been processed
        self._queried_camera_1 = False
        self._rate = rospy.Rate(rate)
        self._robot_name = 'waffle'
        self._velocity_msg = Twist()
        #  gains for the proportional controller
        self._kp_linear = 0.2
        self._kp_angular = 0.2

        #  default velocities for going in a straight line
        self._velocity_msg.linear.x = 0.1
        self._velocity_msg.angular.z = 0.1

        # current pose of the robot
        self._current_x_pos = None
        self._current_y_pos = None
        self._current_orientation = None
        self._initial_orientation = None

        #  used to check whether the goal has been reached
        self._goal_reached = False

        # Publishers
        self._velocity_publisher = rospy.Publisher(
            'cmd_vel', Twist, queue_size=10)
        # Subscribers
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # rospy.Subscriber("/logical_camera/camera_1",
        #                  LogicalCameraImage, self.camera_1_callback)
        # camera1_msg = rospy.wait_for_message('/logical_camera/camera_1', LogicalCameraImage)
        # rospy.loginfo(camera1_msg)

        # part_infos_msg = rospy.wait_for_message('/part_infos', PartInfos)
        # rospy.loginfo(part_infos_msg)

        # self.attach_part("assembly_pump_blue")
        # rospy.sleep(5.0)
        # self.detach_part("assembly_pump_blue")

        self.handle_inputs()
        rospy.spin()

    @staticmethod
    def compute_distance(x1, y1, x2, y2):
        """Compute distance between 2 points

        Args:
            x1 (float): x position of the first point
            y1 (float): y position of the first point
            x2 (float): x position of the second point
            y2 (float): y position of the second point

        Returns:
            float: distance between 2 points
        """
        return sqrt(((x2-x1)**2) + ((y2-y1)**2))

    @staticmethod
    def get_model_pose(model_name="waffle"):
        """
        Get the pose of the turtlebot in the environment.
        This function is used by the method attach_part

        Args:
            model_name (str, optional): Name of the robot model in Gazebo. Defaults to "waffle".

        Raises:
            RuntimeError: Raised when the robot model cannot be retrieved

        Returns:
            Pose: Pose of the robot in the Gazebo world
        """
        poll_rate = rospy.Rate(1)
        for i in range(10):
            model_states = rospy.wait_for_message(
                '/gazebo/model_states', ModelStates, 1)
            if model_name in model_states.name:
                model_pose = model_states.pose[model_states.name.index(
                    model_name)]
                break
            poll_rate.sleep()
        else:
            raise RuntimeError('Failed to get ' + model_name + ' model state')
        return model_pose

    def attach_part(self, part_type):
        """
        Attach a Gazebo model to the turtlebot
        """
        part_name = part_type+"_1"
        rospy.loginfo(part_name)

        pause_physics_client = rospy.ServiceProxy(
            '/gazebo/pause_physics', Empty)
        unpause_physics_client = rospy.ServiceProxy(
            '/gazebo/unpause_physics', Empty)
        spawn_sdf_model_client = rospy.ServiceProxy(
            '/gazebo/spawn_sdf_model', SpawnModel)
        link_attacher_client = rospy.ServiceProxy(
            '/link_attacher_node/attach', Attach)
        delete_model_client = rospy.ServiceProxy(
            'gazebo/delete_model', DeleteModel)
        # Remove from Gazebo
        delete_model_client(part_type+"_0")

        # Compute part pose based on current robot pose
        part_pose = BotController.get_model_pose(self._robot_name)
        rospy.loginfo(part_pose.position.x)
        rospy.loginfo(part_pose.position.y)
        # rospy.logwarn(robot_name)
        # part_pose.position.x -= 0.05
        part_pose.position.y += 0.5
        part_pose.position.z += 0.3
        # part_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))

        model_name = "model://"+part_type+"_ariac"
        rospy.loginfo("Model: {}".format(model_name))
        # Compute spawn model request
        spawn_request = SpawnModelRequest()
        spawn_request.model_name = part_name
        # spawn_request.model_name = "assembly_pump_yellow_0"
        spawn_request.model_xml = '<sdf version="1.6"> \
        <world name="default"> \
            <include> \
                <uri>{0}</uri> \
            </include> \
            </world> \
        </sdf>'.format(model_name)
        spawn_request.robot_namespace = self._robot_name
        spawn_request.initial_pose = part_pose

        # Pause Gazebo
        try:
            pause_physics_client()
        except rospy.ServiceException as e:
            rospy.logerr("Pause physics service call failed: %s", e)

        spawn_sdf_model_client(spawn_request)

        #  Attach the part to the robot
        try:
            link_attacher_client(
                self._robot_name, 'base_footprint', part_name, 'link')
        except rospy.ServiceException as e:
            rospy.logerr("Link attacher service call failed: %s", e)

        #  Unpause Gazebo
        try:
            unpause_physics_client()
        except rospy.ServiceException as e:
            rospy.logerr("Unpause physics service call failed: %s", e)

    def detach_part(self, part_type):
        """
        Detach a model from the turtlebot
        """
        detach_client = rospy.ServiceProxy('/link_attacher_node/detach',
                                           Attach)
        detach_client.wait_for_service()

        part_name = part_type+"_1"

        # Build the detach request
        rospy.loginfo("Detaching part from robot")
        req = AttachRequest()
        req.model_name_1 = self._robot_name
        req.link_name_1 = "base_footprint"
        req.model_name_2 = part_name
        req.link_name_2 = "link"

        detach_client.call(req)

    def camera_1_callback(self, msg: LogicalCameraImage):
        """
        Callback for Topic /logical_camera/camera_1
        """
        if not self._queried_camera_1:
            for part in msg.models:
                camera_part = Part(
                    part.type, "camera_1_"+part.type+"_0_frame", part.pose)
                self._camera_parts.append(camera_part)
            self._queried_camera_1 = True

    def odom_callback(self, msg):
        """
        Callback function for the Topic odom
        Args:
            msg (nav_msgs/Odometry): Odometry message
        """
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self._current_x_pos = msg.pose.pose.position.x
        self._current_y_pos = msg.pose.pose.position.y
        self._current_orientation = euler_from_quaternion(quaternion)

    def get_transform(self, source, target):
        """Get the transform between two frames

        Args:
            source (str): source frame
            target (str): target frame

        Returns:
            geometry_msgs/Pose: The pose of the source frame in the target frame
        """
        tf_buffer = tf2_ros.Buffer(rospy.Duration(3.0))
        tf2_ros.TransformListener(tf_buffer)

        transform_stamped = TransformStamped()

        rospy.logwarn("Source: {}".format(source))
        rospy.logwarn("Target: {}".format(target))

        # for _ in range(5):
        try:
            transform_stamped = tf_buffer.lookup_transform(
                source,
                target,
                rospy.Time(),
                rospy.Duration(1.0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr("Unable to lookup transform")

        pose = Pose()
        pose.position = transform_stamped.transform.translation
        rospy.logwarn("Transform: {}".format(pose))
        pose.orientation = transform_stamped.transform.rotation
        return pose

    def broadcast(self):
        """
        Broadcast a frame on Topic /tf
        """
        br = tf.TransformBroadcaster()
        br.sendTransform((0.1, 0.2, 1),
                         tf.transformations.quaternion_from_euler(0, 0, 3.14),
                         rospy.Time.now(),
                         "new frame",
                         "/world")

    def go_straight(self, distance_to_drive, forward=True):
        """
        Move a robot in a straight line.
        This version uses a TF listener.
        Args:
            distance_to_drive (float): Distance to drive in meter.
            forward (bool, optional): Direction. Defaults to True.
        """
        # self.get_transform('base_footprint', 'odom')

        while self._current_x_pos is None:
            rospy.sleep(1)

        robot_initial_x_pos = self._current_x_pos
        robot_initial_y_pos = self._current_y_pos

        linear_velocity = 0.0
        if forward:
            linear_velocity = 0.3
        elif not forward:
            linear_velocity = -0.3

        # compute the driven distance
        driven_distance = BotController.compute_distance(
            robot_initial_x_pos,
            robot_initial_y_pos,
            self._current_x_pos,
            self._current_y_pos)

        # keep moving the robot until the distance is reached
        while not rospy.is_shutdown():
            if driven_distance <= abs(distance_to_drive):
                driven_distance = BotController.compute_distance(
                    robot_initial_x_pos,
                    robot_initial_y_pos,
                    self._current_x_pos,
                    self._current_y_pos)
                rospy.loginfo("Distance driven: {}".format(driven_distance))
                self.run(linear_velocity, 0)
            else:
                self.run(0, 0)
                break
            self._rate.sleep()

        return True

    def go_to_goal(self, goal_x, goal_y):
        """
        Make the robot reach a 2D goal using a proportional controller
        Args:
            goal_x (float): x position
            goal_y (float): y position
        """
        rospy.loginfo("Go to goal ({}, {})".format(goal_x, goal_y))

        # get position and yaw from transform
        self.get_transform("base_footprint", "odom")

        distance_to_goal = BotController.compute_distance(
            self._current_x_pos, self._current_y_pos, goal_x, goal_y)

        while not rospy.is_shutdown():
            move_cmd = Twist()
            if distance_to_goal > 0.1:
                distance_to_goal = BotController.compute_distance(self._current_x_pos,
                                                                  self._current_y_pos, goal_x, goal_y)
                # compute the heading
                angle_to_goal = atan2(
                    goal_y - self._current_y_pos, goal_x - self._current_x_pos)

                # rospy.loginfo("Distance to goal: {}".format(distance_to_goal))
                # rospy.loginfo("Angle to goal: {}".format(angle_to_goal))

                # Make the robot rotate to face the goal
                if angle_to_goal < 0:
                    angle_to_goal = 2 * pi + angle_to_goal

                # compute relative orientation between robot and goal
                w = angle_to_goal - self._current_orientation[2]
                if w > pi:
                    w = w - 2 * pi

                # proportional control for angular velocity
                move_cmd.angular.z = self._kp_angular * w

                # turtlebot max angular velocity is 2.84 rad/s
                if move_cmd.angular.z > 0:
                    move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
                else:
                    move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

                # proportional control for linear velocity
                # turtlebot max linear velocity is 0.22 m/s
                move_cmd.linear.x = min(
                    self._kp_linear * distance_to_goal, 0.6)

                self._velocity_publisher.publish(move_cmd)
                self._rate.sleep()
            else:
                rospy.loginfo("Goal reached")
                self._goal_reached = True
                self.run(0, 0)
                return True
                # break

    def run(self, linear, angular):
        """
        Publish linear and angular velocities to cmd_vel Topic.
        Args:
            linear (float): linear velocity
            angular (float): angular velocity
        """
        velocity = Twist()
        velocity.linear.x = linear
        velocity.angular.z = angular
        self._velocity_publisher.publish(velocity)

    def myhook(self):
        """
        Function to call when shutting down a Node
        """
        rospy.loginfo("shutdown time!")

    def handle_inputs(self):
        """
        Handle arguments passed to the command line
        """

        # get the action to perform
        # drive straight or go to goal
        action_name = rospy.get_param("~action")

        if action_name == "drive":
            distance = rospy.get_param("~distance")
            if distance > 0:
                rospy.logwarn("distance")
                if self.go_straight(distance, True):
                    rospy.logwarn("Action completed")
                    rospy.on_shutdown(self.myhook)
                    sys.exit(1)
            elif distance < 0:
                if self.go_straight(distance, False):
                    rospy.logwarn("Action completed")
                    rospy.on_shutdown(self.myhook)
                    sys.exit(1)
            else:
                rospy.logerr("Distance not provided")
                rospy.on_shutdown(self.myhook)
                sys.exit(1)
        elif action_name == "goal":
            x = rospy.get_param("~x")
            y = rospy.get_param("~y")
            if x and y:
                if self.go_to_goal(x, y):
                    rospy.logwarn("Action completed")
                    rospy.on_shutdown(self.myhook)
                    sys.exit(1)
            else:
                rospy.logerr("x or y is missing")
                rospy.on_shutdown(self.myhook)
                sys.exit(1)
        elif action_name == "broadcast":
            while not rospy.is_shutdown():
                self.broadcast()
        else:
            rospy.logerr("Unknown action")
            rospy.on_shutdown(self.myhook)
            sys.exit(1)
