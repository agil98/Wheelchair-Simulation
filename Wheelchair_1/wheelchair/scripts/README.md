## Recording IMU data

* The data can be recorded into a rosbag called _imu_ using the command `rosbag record -O imu` followed by the names of the topics to be recorded.

* By running the script _rosbag_to_xlsx.sh_ we transfer the data inside the bag to three xlsx files containing the data for the frame and the two wheels.