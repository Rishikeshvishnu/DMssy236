# Intelligent Object Recognition and Localization System for TIAGo Robot in Gazebo

## Overview

This project implements an integrated robotics system using the TIAGo robot in Gazebo simulation. The system combines:
- **Computer Vision**: Object detection using YOLO (darknet_ros)
- **Knowledge Representation**: Ontology-based reasoning with KnowRob
- **Robot Control**: Navigation and arm manipulation for object interaction

## Prerequisites

- ROS Noetic
- Gazebo simulation environment
- KnowRob framework
- darknet_ros package (YOLO object detection)
- TIAGo robot packages

## Project Structure

```
final_project/
├── launch/
│   ├── gazebo_ssy236.launch      # Gazebo simulation
│   └── reasoning.launch           # Knowledge reasoning system
├── src/
│   ├── reasoning_node             # Main reasoning node
│   ├── learning_node.py           # Object learning interface
│   ├── camera_perception.py       # Camera processing
│   ├── tiago_control_node         # Robot movement controller
│   ├── tiago_arm_control.py       # Arm control for pointing
│   └── test_arm_control.py        # Arm control testing
└── worlds/
    └── kitchen_chalmers.world     # Simulation environment
```

## Setup Instructions

### Step 0: Clean Workspace

Before starting, clean any previous builds:

```bash
cd /home/student/ros/workspaces/ssy236_vishnuri/
rm -rf devel build
```

**Note:** Create a `CATKIN_IGNORE` file (no extension) in any other packages you want to exclude from the build.

### Step 1: Build the Workspace

```bash
cd /home/student/ros/workspaces/ssy236_vishnuri/
source ../knowrob_noetic/devel/setup.bash
catkin_make
```

## Running the System

The system requires multiple terminals running simultaneously. Follow these steps in order:

### Terminal 1: ROS Master
```bash
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
roscore
```

### Terminal 2: Gazebo Simulation
```bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/student/ros/workspaces/ssy236_vishnuri/src/final_project/
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
roslaunch final_project gazebo_ssy236.launch
```

### Terminal 3: Knowledge & Reasoning System
```bash
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
roslaunch final_project reasoning.launch
```

### Terminal 4: Object Detection (YOLO)
```bash
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
roslaunch darknet_ros darknet_ros.launch
```

### Terminal 5: Reasoning Node
```bash
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosrun final_project reasoning_node
```

### Terminal 6: Robot Movement Controller
```bash
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosrun final_project tiago_control_node
```

### Terminal 7: Arm Control
```bash
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosrun final_project tiago_arm_control.py
```

### Terminal 8: Camera Perception
```bash
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosrun final_project camera_perception.py
```

## Usage Examples

### Learning Objects

Run the learning node to select and learn objects:

```bash
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosrun final_project learning_node.py
```

Follow the on-screen prompts to select an object from the list.

### Verifying Objects in Ontology

Check if an object exists in the knowledge base:

```bash
rosservice call /check_object_existence "object_name: 'bottle'"
```

Replace `'bottle'` with any object you want to verify.

### Testing Arm Control

Make the robot point at detected objects:

```bash
cd /home/student/ros/workspaces/ssy236_vishnuri/
source devel/setup.bash
rosrun final_project test_arm_control.py
```

## Task 2: Prolog Predicates

### Checking Object Classes

Check if a class exists in the ontology:

```prolog
?- rdf_has(ssy236Ontology:'Beer', rdf:type, owl:'Class').
true.
```

Check for instances of a class:

```prolog
?- owl_individual_of(I, ssy236Ontology:'Beer').
false.
```

### Adding Object Instances

Add new object instances to the ontology:

```prolog
% Add a beer instance
?- rdf_assert(ssy236Ontology:'beer_1', rdf:type, ssy236Ontology:'Beer').
true.

% Add a pot instance
?- rdf_assert(ssy236Ontology:'pot_1', rdf:type, ssy236Ontology:'Pot').
true.

% Add pan instances
?- rdf_assert(ssy236Ontology:'pan_1', rdf:type, ssy236Ontology:'Pan').
true.

?- rdf_assert(ssy236Ontology:'pan_2', rdf:type, ssy236Ontology:'Pan').
true.
```

### Verification Queries

Verify that instances were added correctly:

```prolog
% Check all pan instances
?- owl_individual_of(I, ssy236Ontology:'Pan').
I = ssy236Ontology:pan_1 ;
I = ssy236Ontology:pan_2 ;
false.

% Check all pot instances
?- owl_individual_of(I, ssy236Ontology:'Pot').
I = ssy236Ontology:pot_1 ;
false.

% Check all beer instances
?- owl_individual_of(I, ssy236Ontology:'Beer').
I = ssy236Ontology:beer_1 ;
false.
```

## Debugging & Monitoring

### View Camera Feed

Monitor the robot's camera and object detection:

```bash
rosrun rqt_image_view rqt_image_view
```

### Check Simulation Time

Verify that the simulation clock is running:

```bash
rostopic echo /clock
```

### Monitor Object Detection

List all YOLO detection topics:

```bash
rostopic list | grep darknet_ros
```

View real-time object detections:

```bash
rostopic echo /darknet_ros/bounding_boxes
```

This will show detected objects with their bounding box coordinates and confidence scores.

## Troubleshooting

### Common Issues

1. **Gazebo doesn't start**: Ensure GAZEBO_RESOURCE_PATH is set correctly
2. **No object detections**: Check if darknet_ros is receiving camera images
3. **KnowRob errors**: Verify that the knowrob workspace is sourced properly
4. **Build errors**: Clean the workspace with `rm -rf devel build` and rebuild

### Verifying System Status

- Check that all required topics are publishing: `rostopic list`
- Monitor node status: `rosnode list`
- View node information: `rosnode info <node_name>`

## Services

The system provides several ROS services:

- `/check_object_existence`: Verify if an object exists in the ontology
- Additional services are available through the reasoning and perception nodes

## Contributing

When contributing to this project:
1. Ensure all terminals are properly sourced
2. Test your changes in simulation before committing
3. Update this README if you add new features or change the workflow

## Acknowledgments

- Course: SSY236
- Institution: Chalmers University of Technology
- TIAGo robot simulation framework
- darknet_ros for object detection
- KnowRob for knowledge representation
