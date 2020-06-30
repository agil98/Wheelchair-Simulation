## Recording IMU data
There are two ways to record and transform the Imu sensor data from quaternion to euler. We can run an additional package that will subscribe to the imu sensor and publish the converted data, or the data can be converted we stop running the simulation using a python script.

### Method 1: Package
* First, we have run the _quat_to_euler_ package which, as its name implies, subscribes to the Imu topic, converts the angles and then publishes it into a new topic.
``` 
rosrun quat_to_euler quat_to_euler.py
```

* Then, by running the script _importData.sh_ we transfer the data inside the bag to three xlsx files, _\_imu_left.xlsx_, _\_imu_frame.xlsx_ and _\_imu_right.xlsx_.

```
rosbag record -O imu /imu_left /imu_right /imu_frame
```

### Method 2: Script
* First,  use the following command to record the messages published by the imu sensors into a file called _imu.bag_.
```
rosbag record -O imu /imu_left_quat /imu_right_quat /imu_frame_quat
```

* Then, by running the script _importData.sh_ we transfer the data inside the bag to three xlsx files, _imu_left_quat.xlsx_, _imu_frame_quat.xlsx_ and _imu_right_quat.xlsx_.

* To convert the angles we use the script _convert_quat_to_euler.py_. This will create three new files, _imu_left.xlsx_,  _imu_right.xlsx_ and _imu_frame.xlsx_ which just contains the converted angles, angular velocity and linear acceleration values of the corresponding wheel.
-Note that the python script requires the _xlsxwriter_ module to be able to add data to a new file.
