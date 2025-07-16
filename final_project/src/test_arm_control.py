#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool, String
from gazebo_msgs.msg import ModelStates
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import do_transform_pose
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
import numpy as np
import tf.transformations  # For quaternions
import time

class TiagoArmController:
    def __init__(self):
        # Wait for valid time (like in C++ example)
        start_time = rospy.get_time()
        timeout = 10.0  # Timeout in seconds

        while rospy.get_time() < start_time + timeout:
            rospy.sleep(0.1)

        # Initialize action client (following C++ pattern)
        self.client = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )

        # Wait for server - same timeout pattern as C++
        iterations = 0
        max_iterations = 3
        while not self.client.wait_for_server(rospy.Duration(2.0)) and iterations < max_iterations:
            rospy.loginfo("Waiting for arm controller server...")
            iterations += 1

        if iterations == max_iterations:
            rospy.logerr("Arm controller action server not available!")
            return

        # Joint names - exactly as in C++ example
        self.joint_names = [
            'arm_1_joint',
            'arm_2_joint',
            'arm_3_joint',
            'arm_4_joint',
            'arm_5_joint',
            'arm_6_joint',
            'arm_7_joint'
        ]

        # Pre-defined poses (from C++ example)
        self.first_pose = [0.2, 0.0, -1.5, 1.94, -1.57, -0.5, 0.0]
        self.pointing_pose = [2.5, 0.2, -2.1, 1.9, 1.0, -0.5, 0.0]

    def generate_pointing_trajectory(self):
        """Generate trajectory exactly like C++ example"""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        # Two waypoints
        goal.trajectory.points = []

        # First waypoint - interim position
        point1 = JointTrajectoryPoint()
        point1.positions = self.first_pose
        point1.velocities = [1.0] * 7  # Non-zero velocity for smooth motion
        point1.time_from_start = rospy.Duration(2.0)
        goal.trajectory.points.append(point1)

        # Second waypoint - pointing position
        point2 = JointTrajectoryPoint()
        point2.positions = self.pointing_pose
        point2.velocities = [0.0] * 7  # Zero velocity to stop at target
        point2.time_from_start = rospy.Duration(4.0)
        goal.trajectory.points.append(point2)

        return goal

    def point_arm(self, target_pose):
        """Execute pointing motion"""
        try:
            goal = self.generate_pointing_trajectory()
            # Set start time 1sec from now (like C++)
            goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
            # Send goal
            rospy.loginfo("Sending pointing goal...")
            self.client.send_goal(goal)
            # Wait for result with timeout
            if self.client.wait_for_result(rospy.Duration(10.0)):
                return self.client.get_state() == actionlib.GoalStatus.SUCCEEDED
            else:
                rospy.logerr("Pointing action timed out!")
                return False
        except Exception as e:
            rospy.logerr(f"Error in point_arm: {e}")
            return False

class PointingControl:
    def __init__(self):
        rospy.init_node('pointing_control_node')
        self.target_object = None
        self.target_pose = None  # To store the target's pose
        self.object_found = False
        self.pointing_complete = False
        self.last_target_update_time = 0
        self.target_update_delay = 1.0  # 1 second delay between updates

        # Initialize arm controller
        self.arm_controller = TiagoArmController()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers
        self.found_sub = rospy.Subscriber(
            '/object_detection_monitor/target_found',
            Bool,
            self.found_callback,
            queue_size=1
        )
        self.target_sub = rospy.Subscriber(
            '/perception/target_object',
            String,
            self.target_callback,
            queue_size=1
        )
        # Subscriber for Gazebo model states to get object pose
        self.model_state_sub = rospy.Subscriber(
            '/gazebo/model_states',
            ModelStates,
            self.model_state_callback,
            queue_size=1
        )

        rospy.loginfo("Pointing control node initialized")

    def model_state_callback(self, msg):
        """Get the target object pose from Gazebo model states"""
        if self.target_object is None:
            return
        try:
            self.target_object = 'coke_can'
            obj_idx = msg.name.index(self.target_object)
            obj_pose = msg.pose[obj_idx]
            # Transform the pose from world frame to base_footprint frame
            self.transform_to_base_frame(obj_pose)
        except ValueError:
            rospy.logwarn(f"Object {self.target_object} not found in Gazebo model states.")

    def transform_to_base_frame(self, obj_pose):
        """Transform object pose from odom frame to base_footprint frame"""
        try:
            # Create a PoseStamped object
            pose_stamped = geometry_msgs.msg.PoseStamped()
            pose_stamped.header.frame_id = 'odom'
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose = obj_pose

            # Transform the pose
            transform = self.tf_buffer.lookup_transform('base_footprint', 'odom', rospy.Time())
            transformed_pose = do_transform_pose(pose_stamped, transform)

            rospy.loginfo(f"Transformed pose: {transformed_pose.pose.position.x}, {transformed_pose.pose.position.y}, {transformed_pose.pose.position.z}")

            # Now use this pose to plan the arm movement
            self.plan_arm_movement(transformed_pose.pose)
        except (tf2_ros.TransformException, tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error transforming object pose: {e}")

    def plan_arm_movement(self, transformed_pose):
        """Plan arm movement to point at target"""
        try:
            # Create MoveGroup once
            if not hasattr(self, 'move_group'):
                self.move_group = MoveGroupCommander("arm")
            rospy.sleep(1.0)  # Give time to initialize

            # Calculate pointing pose
            target_pos = transformed_pose.position
            # Calculate direction vector
            dx = target_pos.x
            dy = target_pos.y
            dz = target_pos.z
            # Scale distance (don't try to reach target, just point)
            distance = 0.6  # Keep arm closer, about 60cm extended
            scale = distance / np.sqrt(dx*dx + dy*dy + dz*dz)
            # Create pointing pose
            pointing_pose = geometry_msgs.msg.Pose()
            pointing_pose.position.x = dx * scale
            pointing_pose.position.y = dy * scale
            pointing_pose.position.z = 0.5  # dz * scale

            # Calculate orientation to point AT target
            pointing_vector = np.array([
                target_pos.x - pointing_pose.position.x,
                target_pos.y - pointing_pose.position.y,
                target_pos.z - pointing_pose.position.z
            ])
            direction = pointing_vector / np.linalg.norm(pointing_vector)  # [dx, dy, dz]
            direction_norm = np.linalg.norm(direction)
            if direction_norm > 0:
                direction = direction / direction_norm
            # Create rotation matrix pointing along direction
            z_axis = direction
            y_axis = np.cross([0, 0, 1], z_axis)
            if np.linalg.norm(y_axis) < 1e-6:
                y_axis = [0, 1, 0]
            y_axis = y_axis / np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, z_axis)
            # Convert to quaternion
            rotation_matrix = np.vstack((x_axis, y_axis, z_axis)).T
            quat = tf.transformations.quaternion_from_matrix(
                np.vstack([np.hstack([rotation_matrix, [[0], [0], [0]]]),
                           [0, 0, 0, 1]])
            )
            pointing_pose.orientation.x = quat[0]
            pointing_pose.orientation.y = quat[1]
            pointing_pose.orientation.z = quat[2]
            pointing_pose.orientation.w = quat[3]

            # Plan and execute
            self.move_group.set_pose_target(pointing_pose)
            success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            return success
        except Exception as e:
            rospy.logerr(f"Error planning movement: {e}")
            return False

    def target_callback(self, msg):
        """Record target object with debouncing"""
        current_time = time.time()
        if current_time - self.last_target_update_time > self.target_update_delay:
            if self.target_object != msg.data:
                self.target_object = msg.data
                self.pointing_complete = False
            rospy.loginfo(f"New target object: {self.target_object}")
            self.last_target_update_time = current_time

    def found_callback(self, msg):
        """Handle object found event"""
        if msg.data and not self.pointing_complete:
            rospy.loginfo("Object found - waiting for pose update")
            self.object_found = True

def main():
    try:
        node = PointingControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()