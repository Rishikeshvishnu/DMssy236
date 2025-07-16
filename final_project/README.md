
#####################################################################################################################
#### GROUP 11 - PROJECT ####
#####################################################################################################################
Mathanesh Vellingiri Ramasamy
Rishikesh Vishnu Sivakumar

#####################################################################################################################
#### SETUP: OPEN VSCode ####
##################################################################################################################### 

#### STEP 0: CATKIN_IGNORE
Goto the other packages inside your workspace and create a new empty file (without an extension) called
“CATKIN_IGNORE”, remove the folders build/ and devel/ and then proceed.

# To remove the folders build and devel
rm -rf devel build

# IN TERMINAL 1: Setup & Compilation
cd /home/student/ros/workspaces/ssy236_vishnuri/
source ../knowrob_noetic/devel/setup.bash
catkin_make

# IN TERMINAL 2: Start ROS Master
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
roscore

# IN TERMINAL 3: Launch Gazebo Simulation
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/student/ros/workspaces/ssy236_vishnuri/src/final_project/
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
roslaunch final_project gazebo_ssy236.launch

# IN TERMINAL 4: Launch Knowledge & Reasoning System
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
roslaunch final_project reasoning.launch

# IN TERMINAL 5: Launch Perception System (from darknet_ros package)
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
roslaunch darknet_ros darknet_ros.launch

# IN TERMINAL 6: Run Reasoning Node
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosrun final_project reasoning_node

# IN TERMINAL 7: Select Object & Trigger Perception (Choose the object from the list)
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosrun final_project learning_node.py

# IN TERMINAL 8: Verify Object in Ontology (Use the same object as above)
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosservice call /check_object_existence "object_name: 'bottle'"

# IN TERMINAL 9: Run Robot Movement Controller
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosrun final_project tiago_control_node

# IN TERMINAL 10: Run Arm Control for Pointing
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosrun final_project tiago_arm_control.py

# IN TERMINAL 11: Run Camera Perception
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosrun final_project camera_perception.py

# IN TERMINAL 12: Run test_arm_control.py - will make the robot point at the object
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosrun final_project test_arm_control.py 


# ############################################################################
# TASK 2: PROLOG PREDICATES:

# CHECKING INITIALLY:

?- rdf_has(ssy236Ontology:'Beer', rdf:type, owl:'Class').
true.

?- owl_individual_of(I, ssy236Ontology:'Beer').
false.

# ADDING PREDICATES:

?- rdf_assert(ssy236Ontology:'beer_1', rdf:type, ssy236Ontology:'Beer').
true.

?- rdf_assert(ssy236Ontology:'pot_1', rdf:type, ssy236Ontology:'Pot').
true.

?- rdf_assert(ssy236Ontology:'pan_1', rdf:type, ssy236Ontology:'Pan').
true.

?- rdf_assert(ssy236Ontology:'pan_2', rdf:type, ssy236Ontology:'Pan').
true.

# VERIFICATION:
?- owl_individual_of(I, ssy236Ontology:'Pan').
I = ssy236Ontology:pan_1 ;
I = ssy236Ontology:pan_2 ;
false.

?- owl_individual_of(I, ssy236Ontology:'Pot').
I = ssy236Ontology:pot_1 ;
false.

?- owl_individual_of(I, ssy236Ontology:'Beer').
I = ssy236Ontology:beer_1 ;
false.

# ############################################################################
# DEBUGGING: Monitor the topic /darknet_ros/bounding_boxes to check if Detected Object Matches Target
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosrun rqt_image_view rqt_image_view

CHECK CLOCK:
rostopic echo /clock

CHECK YOLO:
rostopic list | grep darknet_ros

CHECK DETECTIONS BY YOLO:
rostopic echo /darknet_ros/bounding_boxes

# ############################################################################
