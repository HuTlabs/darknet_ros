# YOLO ROS: Real-Time Object Detection for ROS

# Installations

1) Create a workspace name yolo and git clone this package into that package

   mkdir ~/yolo

   cd yolo

   mkdir src

   cd src

   git clone https://github.com/HuTlabs/darknet_ros.git

   cd ..

   catkin_make

2) After doing catkin_make, follow this path 

   cd src/darknet_ros/darknet_ros/yolo_network_config/weights
   
   The weights should be download with the help of how_to_download_weights.txt file and kept in that folder itself.
   
# Working

For camera, we will obtain rbg camera topic and depth camera topic, which should make a note of these.

1) Need to change the topic in 2 places

   i) cd src/darknet_ros/darknet_ros/config/
   
          In ros.yaml file, change the topic of camera_reading and camera_depth.
   
   ii) cd src/darknet_ros/darknet_rod/src

           In the YoloObjectDetector.cpp file, change the rbg and depth image topic names on line no 44, 45.
   
2) Launch the detection file

   cd ~/yolo
   
   catkin_make
   
   source devel/setup.bash
   
   roslaunch darknet_ros yolo_v3.launch

3) Run compare.py file, which is presented in src folder.
