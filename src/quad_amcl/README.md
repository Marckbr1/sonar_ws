# quad_amcl
This folder should be pasted in the src folder of your ROS workspace.

Download the testing dataset from https://drive.google.com/file/d/1CTItLDgSqoSi3j5pz3l-eigY10JzQelg/view?usp=sharing and past on folder quad_amcl/imgs/

Compile your workspace using catkin build or catkin_make.

Run the test launch:
$ roslaunch quad_amcl test.launch
You must see the quadruplet print some msgs telling that images were received.
The dummy quadruplet script is in quad_amcl/scripts/quadruplet.py

Run quad_amcl full:

1- Download bag file https://drive.google.com/file/d/1Krbi6Tm7zDSZzS5JUeeiB7tw5KaRQSYK/view?usp=sharing and past on folder bags.
2- On your workspace folder, build the project:
 $ catkin build
3- On workspace folder, source the ws to update new nodes.
 $ source devel/setup.bash
3- Run command:
 $ roslaunch quad_amcl run.launch
