Package img_proc
−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
## Brightness node
### Description
This node determines the average brightness of the image and checks whether a light is turned on or of compared to some threshold
### Inputs
‘/ image‘
Type: sensor_msgs/msg/Image
### Outputs
‘/ brightness_estimate‘
Type: std_msgs/msg/String
A string with node's decision whether the light is on or off.
### Run
In a terminal run the following commands:
‘ ros2 run img_proc brightness ‘
### Parameters
int ‘threshold‘ : The average brightness of an image above which the light is considered on
### Core components
‘ imageCallback()‘: Receives the image, calculates it's average brightness and compares to the threshold

## Color tracker node
### Description
This nodes detects the brightest regions in a specific channel of an image and finds their common center of mass and bounding box
### Inputs
‘/ image‘
Type: sensor_msgs/msg/Image
### Outputs
‘/ tracked_CoM‘
Type: geometry_msgs/msg/Point
Coordinates of the center of mass of white pixels on the image after threshold. z coordinate is always zero

‘/ tracked_bbox‘
Type: vision_msgs/msg/BoundingBox2D
Coordinates of the bounding box of all white pixels after threshold. Contains coordinates of the center of the bounding box and it's size
### Run
In a terminal run the following commands:
‘ ros2 run img_proc color_tracker ‘
### Parameters
string ‘channel‘ : Which image channel to use. "b" - blue, "g" - green, "r" - red, "k" - grayscale
int ‘threshold‘ : The value above which the tracked object's pixels have to be in the chosen channel
bool ‘show_image‘ : Whether to show the image after binarization with indicated center of mass and bounding box
### Core components
‘ imageCallback()‘: Receives the image and calls processing functions
‘ extractChannel()‘: Retrieves the chosen channel from the image
‘ binaryze()‘: Applies threshold, reduces noise and fills holes
