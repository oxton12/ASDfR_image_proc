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
ros2 run img_proc color_tracker --ros-args --params-file src/image_proc/config/color_tracker.yaml
Location in code: imageCallback() in color_tracker.cpp

To view the center of mass position in the third run
ros2 topic echo \tracked_CoM

Or to view boinding box message
ros2 topic echo \tracked_bbox


Assignment 1.2
−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
1. Open two new terminals, in the first run
ros2 run my_custom_package node2
Location in code: colour_detection() function in image_analysis.cpp
In the second run
ros2 run my_other_custom_package node1
Location in code: steer_calculation () function in steer ()
Assignment 25.3
−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
1. Run
ros2 launch my_custom_package start_all_nodes.launch.py
and send a velocity
ros2 topic pub /my_topic example_msgs/msg/Float64 "{data: 2.0}"
Assignment connects node 1, 2 and node from other−package together.