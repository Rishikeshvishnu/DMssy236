#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from darknet_ros_msgs.msg import BoundingBoxes
import tf.transformations as tf_trans
from final_project.srv import SetLocation, SetTarget, SetLocationResponse, SetTargetResponse, MovementCommand

class PerceptionNode:
    def __init__(self):
        rospy.init_node('perception_node')

        # Core state management
        self.target_object = None
        self.target_location = None
        self.detection_active = False
        self.object_found = False

        # Initialize components
        self.bridge = CvBridge()

        # Movement client - wait for service
        rospy.loginfo("Waiting for movement service...")
        rospy.wait_for_service('movement_command')
        self.movement_client = rospy.ServiceProxy('movement_command', MovementCommand)

        # Publishers
        self.debug_image_pub = rospy.Publisher("/perception/debug_image", Image, queue_size=1)
        self.state_pub = rospy.Publisher("/perception/state", String, queue_size=1)
        self.found_pub = rospy.Publisher('/object_detection_monitor/target_found', Bool, queue_size=1)

        self.object_name_pub = rospy.Publisher('/perception/target_object', String, queue_size=1)


        # Service servers
        self.location_srv = rospy.Service('/perception/set_location', SetLocation, self.handle_location)
        self.target_srv = rospy.Service('/perception/set_target', SetTarget, self.handle_target)

        # Subscribers
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.image_callback, queue_size=1)
        self.bbox_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detection_callback, queue_size=1)

        rospy.loginfo("Perception node initialized and ready for testing")

    def handle_location(self, req):
        """
        Handle new location command
        Returns: True if movement succeeded
        """
        rospy.loginfo(f"Received new location target: {req.location_name}")
        # Update state
        self.target_location = req.location_name
        self.object_found = False
        self.detection_active = False  # Disable detection during movement

        # Request movement
        try:
            response = self.movement_client(req.pose)
            if response.success:
                rospy.loginfo("Movement completed successfully")
                # Only activate detection if we have both location and target
                self.detection_active = (self.target_object is not None)
                self._publish_state("AT_LOCATION")
            else:
                rospy.logerr(f"Movement failed: {response.message}")
                self._publish_state("MOVEMENT_FAILED")
        except rospy.ServiceException as e:
            rospy.logerr(f"Movement service call failed: {e}")
            self._publish_state("SERVICE_ERROR")
            return SetLocationResponse(False)

        return SetLocationResponse(True)

    def handle_target(self, req):
        """
        Set new target object
        Returns: True after updating target
        """
        rospy.loginfo(f"New target object: {req.object_name}")
        # Update state
        self.target_object = req.object_name
        self.object_found = False

        # Only activate if we're at the location
        self.detection_active = (self.target_location is not None)
        if self.detection_active:
            self._publish_state("SEARCHING")
        else:
            self._publish_state("WAITING_FOR_LOCATION")

        return SetTargetResponse(True)

    # def detection_callback(self, msg):
    #     """Process darknet detections"""
    #     if not self.detection_active:
    #         return

    #     # Look for target object
    #     found = False
    #     for box in msg.bounding_boxes:
    #         if box.Class.lower() == self.target_object.lower():
    #             found = True
    #             self.found_pub.publish(Bool(found))
    #             self.object_name_pub.publish(String(self.target_object))
    #             if not self.object_found:  # Only log first detection
    #                 rospy.loginfo(f"Found target object {self.target_object}!")
    #                 self.object_found = True
    #             break

    #     # Publish results
        
    #     # self._publish_state("FOUND" if found else "SEARCHING")

    def detection_callback(self, msg):
        """Process darknet detections"""
        rospy.loginfo(f"Received YOLO detection message with {len(msg.bounding_boxes)} bounding boxes")

        if not self.detection_active:
            rospy.logwarn("Detection is not active. Ignoring detections.")
            return

        detected = False
        for box in msg.bounding_boxes:
            rospy.loginfo(f"Detected: {box.Class} with probability {box.probability:.2f}")
            if box.Class.lower() == self.target_object.lower():
                detected = True
                rospy.loginfo(f"Target object {self.target_object} FOUND with probability {box.probability:.2f}")
                self.found_pub.publish(Bool(True))
                self.object_name_pub.publish(String(self.target_object))
                self.object_found = True
                break  # Stop checking after the first match

        if not detected:
            rospy.logwarn(f"Target object {self.target_object} NOT found in detected objects.")

        # Check if object was found and trigger ontology update
        if self.object_found:
            rospy.wait_for_service("arm_point")
            point_client = rospy.ServiceProxy("arm_point", Bool)
            point_client(True)

            rospy.wait_for_service("assert_knowledge")
            add_to_ontology = rospy.ServiceProxy("assert_knowledge", UpdateObjectList)
            add_to_ontology(self.target_object)

            rospy.loginfo(f"Updated ontology with new object: {self.target_object}")

            response = add_to_ontology(self.target_object)
    
            if response.confirmation:
                rospy.loginfo(f"Successfully updated ontology with object: {self.target_object}")
            else:
                rospy.logerr(f"Ontology update failed for {self.target_object}")



    def image_callback(self, msg):
        """Process camera images for visualization"""
        try:
            # Convert image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width = frame.shape[:2]

            # Draw status overlay
            status = self._get_status_text()
            cv2.putText(
                frame,
                status,
                (10, height - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            # Publish debug visualization
            debug_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")

    def _get_status_text(self):
        """Generate status text for visualization"""
        if not self.target_location:
            return "Waiting for location..."
        if not self.target_object:
            return "Waiting for target object..."
        if not self.detection_active:
            return "Detection inactive"
        if self.object_found:
            return f"Found {self.target_object}!"
        return f"Searching for {self.target_object}..."

    def _publish_state(self, state):
        """Publish node state"""
        self.state_pub.publish(String(state))

def main():
    try:
        node = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()