# SpCoMapping

### Overview  
SpCoMapping for ROS indigo, Ubuntu 14.04.

### Description

###### Preparation

1. Download the dataset from `https://github.com/EmergentSystemLabStudent/Dataset_of_SpCoMapping.git`

2. put in the `/data/DATASET/`

###### Training

1. `roslaunch spco_mapping amcl.launch`

2. `./spco_mapping.sh`

3. `rostopic pub /spco/read_dataset std_msgs/Int8 1`  
The integer number has no meaning.

4. `rostopic pub /spco/start_learning std_msgs/String "start"`  
The string has no meaning.

### Install

###### In the catkin_workspace

1. `git clone https://github.com/EmergentSystemLabStudent/spco_mapping.git`

2. `catkin_make`

3. `sudo apt-get update`

4. `sudo apt-get install ros-indigo-amcl ros-indigo-map-server`
