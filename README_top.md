Assignment 1.1
−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
### Image processing with ROS2
Open two terminals, in the first run
ros2 run cam2image_vm2ros cam2image

In the second run
ros2 run image_tools showimage

Stop the process in the first terminal and run
ros2 run cam2image_vm2ros cam2image −−ros-args −p depth:=1 −p history:=keep_last

### Adding a brightness node
Open three terminals, in the first run
ros2 run cam2image_vm2ros cam2image

In the second run
ros2 run img_proc brightness
Location in code: imageCallback() in brightness.cpp

In the third run
ros2 topic echo /brightness_estimate

### Adding ROS2 parameters
Open four terminals, in the first run
ros2 run cam2image_vm2ros cam2image

In the second run
ros2 run img_proc brightness --ros-args -p threshold:=250
Location in code: imageCallback() in brightness.cpp

In the third run
ros2 topic echo /brightness_estimate

In the fourth change the parameter by running
ros2 param set /brightness threshold 100

### Simple object-position indicator
Open three terminals, in the first run
ros2 run cam2image_vm2ros cam2image

In the second run 
ros2 run img_proc color_tracker
Location in code: imageCallback() in color_tracker.cpp

Or to use parameters listed in yaml file
ros2 run img_proc color_tracker --ros-args --params-file src/img_proc/config/color_tracker.yaml
Location in code: imageCallback() in color_tracker.cpp

To view the center of mass position in the third run
ros2 topic echo \tracked_CoM

Or to view boinding box message
ros2 topic echo \tracked_bbox


Assignment 1.2
−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
### Unit test the sequence controller using the RELbot Simulator
In config/setpoints.yaml change setpoints to true
Open a terminal and run
ros2 launch seq_contr seq_contr.launch.py
Location in code: sequenceController() in setpoint_sequence_node.cpp

### Integration of image processing and RELbot Simulator
In config/setpoints.yaml change setpoints to false and mode to false
In seq_contr.launch.py change remapping of 'image' topic in 'color_tracker' node to 'image'
Open a terminal and run
ros2 launch seq_contr seq_contr.launch.py
Location in code: sequenceController() in setpoint_sequence_node.cpp

### Closed loop control of the RELbot Simulator
In config/setpoints.yaml change setpoints to false and mode to true
In seq_contr.launch.py change remapping of 'image' topic in 'color_tracker' node to 'output/moving_camera'
Open a terminal and run
ros2 launch seq_contr seq_contr.launch.py
Location in code: sequenceController() in setpoint_sequence_node.cpp
