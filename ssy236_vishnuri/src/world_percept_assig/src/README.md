- open a terminal
- source the workspace
- rosrun world_percept_assig map_generator_node

- open a new terminal
- source the workspace 
- rosservice call /update_object_list '{object_pose: {position: {x: 1.0, y: 2.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, object_name: "cube1"}' (example values, but the format is correct)
- the above command should give confirmation: True in this terminal and should display the pose and object in the first terminal