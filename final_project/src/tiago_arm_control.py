#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib
import numpy as np

class TiagoArmController:
    def __init__(self):
        """Initialize the arm controller"""
        # Set up TF for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize action client - exactly like C++ version
        self.arm_client = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        # Wait for server with timeout like C++ version
        iterations = 0
        max_iterations = 3
        while not self.arm_client.wait_for_server(rospy.Duration(2.0)) and iterations < max_iterations:
            rospy.logdebug("Waiting for the arm controller action server...")
            iterations += 1
            
        if iterations == max_iterations:
            raise RuntimeError("Arm controller action server not available!")
            
        # Define arm joint configuration
        self.arm_joints = [
            'arm_1_joint',  # Shoulder pan
            'arm_2_joint',  # Shoulder lift
            'arm_3_joint',  # Upper arm roll
            'arm_4_joint',  # Elbow flex
            'arm_5_joint',  # Forearm roll
            'arm_6_joint',  # Wrist flex
            'arm_7_joint'   # Wrist roll
        ]
        
        # Common poses we might use
        self.home_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ready_pose = [0.0, 0.3, 0.0, -0.5, 0.0, 0.0, 0.0]
        
    def generate_pointing_trajectory(self, target_x, target_y, target_z):
        """
        Generate a smooth trajectory to point at target
        
        Args:
            target_x, target_y, target_z: Target point coordinates
        
        Returns:
            FollowJointTrajectoryGoal with waypoints
        """
        goal = FollowJointTrajectoryGoal()
        
        # Joint names must match robot
        goal.trajectory.joint_names = self.arm_joints
        
        # Calculate pointing angles
        dx = target_x 
        dy = target_y
        dz = target_z - 0.5  # Adjust for arm base height
        
        # Convert target to angles
        pan = np.arctan2(dy, dx)  # Horizontal angle
        distance = np.sqrt(dx*dx + dy*dy)
        tilt = np.arctan2(dz, distance)  # Vertical angle
        
        # Create three waypoint trajectory
        goal.trajectory.points = []
        
        # 1. Intermediate safe position
        intermediate = JointTrajectoryPoint()
        intermediate.positions = self.ready_pose
        intermediate.velocities = [0.4] * 7  # Conservative speed
        intermediate.time_from_start = rospy.Duration(2.0)
        goal.trajectory.points.append(intermediate)
        
        # 2. Pre-pointing position
        pre_point = JointTrajectoryPoint()
        pre_point.positions = [
            pan * 0.7,    # 70% of final pan
            0.4 + tilt * 0.5,  # Partial lift
            0.0,    # Keep arm roll neutral
            -0.8,   # Slight elbow bend
            0.0,    # Keep forearm neutral
            -0.2,   # Slight wrist angle
            0.0     # Keep wrist roll neutral
        ]
        pre_point.velocities = [0.3] * 7  # Slower as we near target
        pre_point.time_from_start = rospy.Duration(3.0)
        goal.trajectory.points.append(pre_point)
        
        # 3. Final pointing position
        final_point = JointTrajectoryPoint()
        final_point.positions = [
            pan,    # Full pan to target
            0.5 + tilt,  # Full tilt for pointing
            0.0,    # Keep roll neutral
            -1.0,   # Extend elbow slightly
            0.0,    # Keep forearm neutral
            -0.3,   # Angle wrist for pointing
            0.0     # Keep wrist roll neutral
        ]
        final_point.velocities = [0.0] * 7  # Come to complete stop
        final_point.time_from_start = rospy.Duration(4.0)
        goal.trajectory.points.append(final_point)
        
        return goal
        
    def execute_trajectory(self, goal):
        """
        Execute a trajectory goal with proper timing
        
        Args:
            goal: FollowJointTrajectoryGoal to execute
        
        Returns:
            bool: True if trajectory completed successfully
        """
        # Start trajectory 1s from now (like C++ version)
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
        
        # Send goal to action server
        self.arm_client.send_goal(goal)
        
        # Wait for completion with timeout
        finished = self.arm_client.wait_for_result(rospy.Duration(10.0))
        
        if not finished:
            rospy.logerr("Failed to complete trajectory")
            return False
            
        return self.arm_client.get_state() == actionlib.GoalStatus.SUCCEEDED
        
    def point_at_coords(self, x, y, z, frame_id="base_footprint"):
        """
        Point arm at coordinates with proper trajectory
        
        Args:
            x, y, z: Target coordinates
            frame_id: Reference frame for coordinates
            
        Returns:
            bool: True if pointing succeeded
        """
        try:
            # Transform point to arm's reference frame if needed
            if frame_id != "base_footprint":
                point = PointStamped()
                point.header.frame_id = frame_id
                point.header.stamp = rospy.Time(0)
                point.point.x = x
                point.point.y = y
                point.point.z = z
                
                transformed = self.tf_buffer.transform(
                    point, 
                    "base_footprint",
                    rospy.Duration(1.0)
                )
                x = transformed.point.x
                y = transformed.point.y
                z = transformed.point.z
            
            # Generate and execute trajectory
            goal = self.generate_pointing_trajectory(x, y, z)
            return self.execute_trajectory(goal)
            
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform failed: {e}")
            return False

    def move_to_home(self):
        """
        Move arm to home position
        
        Returns:
            bool: True if movement succeeded
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = self.home_pose
        point.velocities = [0.0] * 7
        point.time_from_start = rospy.Duration(2.0)
        
        goal.trajectory.points = [point]
        return self.execute_trajectory(goal)