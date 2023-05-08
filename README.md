# Pan-Tilt camera Visual Servoing
THis repository implements a pan-tilt anology on a rrbot and performs visual servoing to follow a red object.
## Details of implementation
### . rrbot configuration
### . Object detection
### . Control law

## Installation
Go to src folder of your workspace

`cd ~/catkin_ws/src`

Download repository

`github clone https://github.com/TSainathPolisetty/VisualServoing.git`

Catkin make the workspace

`cd ..`

`catkin_make`

Source the workspace

`source devel/setup.bash`

There are three nodes to run. 
1) Simulation
2) Object detection
3) Visual Servoing

1. Simulation

`roslaunch rrbot_gazebo rrbot_world_with_scout.launch`

2. Object detection
In a new terminal

`cd ~/catkin_ws`

`source devel/setup.bash`

`rosrun waypoint_pkg obj_track.py`

Before opening the next node, for visualization, you can visualize the rrbot camera and object detection output in rqt.
Run `rqt` in a new terminal.
Add 2 Image View plugins and stream from the below topics

/rrbot/camera1/image_raw

and 

/object_detector/image_with_box


3. Visual Servoing
In a new terminal

`cd ~/catkin_ws`

`source devel/setup.bash`

`rosrun waypoint_pkg vs_for_rrbot.py`

You can optionally run teleop_keyword to control the red UGV.

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`


### Demo of Visual Servoing with RRBOT(Pan-Tilt)`
[![Watch the Video]()](https://youtu.be/WAG4dLzN1zk)



