#!/usr/bin/env python3

import rospy
import json
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import String
from final_project.srv import checkObjExistence, SetLocation, SetTarget, MovementCommand
from kitchen_object_locator import KitchenObjectLocator

class LearningNode:
    def __init__(self):

        # Initialize ROS node
        rospy.init_node('learning_node')
        rospy.loginfo("Initializing Learning Node...")

        # Initialize available kitchen objects
        self.kitchen_objects = {
            i: obj for i, obj in enumerate([
                "beer_1", "bottle", "water", "glass", "cup",
                "plate", "bowl", "mug", "pot", "pan",
                "milk", "cheese", "yogurt", "butter", "eggs",
                "bread", "fruit", "vegetables", "trash", "waste"
            ], 1)
        }
        
        # Store location poses from world file
        self.location_poses = {
            "table": {
                "x": 2.14935,
                "y": -2.84383,
                "z": 1.0,
                "orientation_w": 1.0
            },
            "cupboard": {
                "x": 1.79675,
                "y": 1.43709,
                "z": 0.8,
                "orientation_w": 1.0
            },
            "counter": {
                "x": -1.18323,
                "y": -1.50681,
                "z": 0.755,
                "orientation_w": 1.0
            },
            "trash_can": {
                "x": 4.17555,
                "y": -1.82032,
                "z": 0.0,
                "orientation_w": 1.0
            }
        }
        
        self.target_object = None
        self.object_locator = KitchenObjectLocator()
        
        # Service Clients
        rospy.loginfo("Waiting for required services...")
        rospy.wait_for_service("check_object_existence")
        rospy.wait_for_service("perception/set_location")
        rospy.wait_for_service("perception/set_target")
        rospy.wait_for_service("movement_command")

        self.client_check_obj_existence = rospy.ServiceProxy("check_object_existence", checkObjExistence)
        self.client_set_location = rospy.ServiceProxy("perception/set_location", SetLocation)
        self.client_set_target = rospy.ServiceProxy("perception/set_target", SetTarget)
        self.client_move_robot = rospy.ServiceProxy("movement_command", MovementCommand)

        # Get user's target object selection and start the process
        self.display_options()
        self.process_target_object()

        rospy.loginfo(f"Learning node initialized with target object: {self.target_object}")

        # Set up publishers
        self.target_pub = rospy.Publisher(
            "/perception/target_object",
            String,
            queue_size=1
        )

        self.locdetails_pub = rospy.Publisher(
            "/perception/locdetails",
            String,
            queue_size=1
        )

        # Set up service client
        self.srv_check_obj_exist = 'check_object_existence'
        self.client_check_obj_existence = rospy.ServiceProxy(
            self.srv_check_obj_exist, 
            checkObjExistence
        )
        
        # Wait for reasoning node's service
        rospy.loginfo("Waiting for check_object_existence service...")
        try:
            rospy.wait_for_service(self.srv_check_obj_exist, timeout=60.0)
        except rospy.ROSException:
            rospy.logerr("Service check_object_existence not available after 60 seconds")
            return
            
        # Process target object
        self.process_target_object()

    def get_pose_from_location(self, location_name):
        """Convert location data to ROS Pose message"""
        if location_name not in self.location_poses:
            rospy.logerr(f"Unknown location: {location_name}")
            return None
            
        pose = Pose()
        loc_data = self.location_poses[location_name]
        pose.position.x = loc_data["x"]
        pose.position.y = loc_data["y"]
        pose.position.z = loc_data["z"]
        pose.orientation.w = loc_data["orientation_w"]
        return pose

    # def process_target_object(self):
    #     """Handle object lookup and location prediction"""
    #     try:
    #         # First check if object exists in ontology
    #         response = self.client_check_obj_existence(self.target_object)
            
    #         if response.exists:
    #             rospy.loginfo(f"Object {self.target_object} exists in ontology")
    #             if response.location:
    #                 # Get object's location from ontology response
    #                 location_name = response.location
    #                 rospy.loginfo(f"Object is on: {location_name}")
                    
    #                 # Get pose for this location
    #                 coordinates = self.get_pose_from_location(location_name)
    #                 if coordinates:
    #                     self.publish_location_details(location_name, coordinates)
    #                 else:
    #                     rospy.logwarn("Could not get coordinates for known location")
    #             else:
    #                 rospy.logwarn("Object exists but location unknown")
    #         else:
    #             rospy.loginfo(f"Object {self.target_object} not in ontology - using ML prediction")
                
    #             # Use ML to predict location
    #             # predicted_location = self.object_locator.predict_location(self.target_object)

    #             # ADDED: Passing all features (`object_type` argument is handled inside `predict_location`)
    #             predicted_location = self.object_locator.predict_location(
    #                 self.target_object,
    #                 weight_category=1,
    #                 size_category=1,
    #                 is_edible=0,
    #                 temp_sensitive=0,
    #                 container_type=0,
    #                 usage_freq=1
    #             )

    #             rospy.loginfo(f"Predicted location: {predicted_location}")
                
    #             # Get coordinates for predicted location
    #             coordinates = self.get_pose_from_location(predicted_location)
    #             if coordinates:
    #                 self.publish_location_details(predicted_location, coordinates)
    #             else:
    #                 rospy.logerr("ML predicted an invalid location")

    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Service call failed: {e}")

    def process_target_object(self):
        """Check ontology, predict location if necessary, and initiate perception"""
        response = self.client_check_obj_existence(self.target_object)

        if response.exists and response.location:
            location_name = response.location
        else:
            location_name = self.object_locator.predict_location(self.target_object)

        pose = self.get_pose_from_location(location_name)
        if not pose:
            rospy.logerr("Invalid location pose")
            return

        # Send location to perception node
        rospy.loginfo(f"Sending {location_name} and target {self.target_object} to perception system")
        self.client_set_location(location_name, pose)
        self.client_set_target(self.target_object)

        # Move robot to location
        move_response = self.client_move_robot(pose)
        if not move_response.success:
            rospy.logerr(f"Failed to move: {move_response.message}")
            return
        
        rospy.loginfo("Robot has reached the target location")


    def publish_location_details(self, location_name, coordinates):
        """Publish location name and coordinates"""
        locdetails_msg = {
            "location_name": location_name,
            "coordinates": {
                "position": {
                    "x": coordinates.position.x,
                    "y": coordinates.position.y,
                    "z": coordinates.position.z
                },
                "orientation": {
                    "w": coordinates.orientation.w
                }
            }
        }
        self.locdetails_pub.publish(json.dumps(locdetails_msg))
        rospy.loginfo("Published location details")
    
    def display_options(self):
        """Display and handle object selection"""
        print("\n=== Kitchen Object Selection ===")
        print("\n=== Choose one from below ===")
        
        for num, obj in self.kitchen_objects.items():
            print(f"{num:2d}: {obj}")
        
        while True:
            try:
                selection = int(input(f"\nSelect an object (1-{len(self.kitchen_objects)}): "))
                if selection in self.kitchen_objects:
                    self.target_object = self.kitchen_objects[selection]
                    print(f"Selected object: {self.target_object}")
                    break
                else:
                    print("Invalid selection. Please try again.")
            except ValueError:
                print("Invalid input. Please enter a number.")

if __name__ == '__main__':
    try:
        node = LearningNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass