## Recording IMU data 
* First,  use the following command to record the messages published by the imu sensors into a file called _imu.bag_.
```
rosbag record -O imu /imu_left /imu_right /imu_frame
```

* Then, by running the script _importData.sh_ we transfer the data inside the bag to three xlsx files, _Quat_imu_left.xlsx_, _Quat_imu_frame.xlsx_ and _Quat_imu_right.xlsx_.

* The imu sensors represent angles using quaternions and includes additional data like odomnetry covariance values. To convert the angles we use the script _convert_quat_to_euler.py_. This will create three new files, _imu_left.xlsx_,  _imu_right.xlsx_ and _imu_frame.xlsx_ which just contains the converted angles, angular velocity and linear acceleration values of the corresponding wheel.
-Note that the python script requires the _xlsxwriter_ module to be able to add data to a new file.
