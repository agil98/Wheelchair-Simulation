## Recording IMU data 
*First,  use the following command to record the messages published by the imu sensors into a file called _imu.bag_.
```
rosbag record -O imu /imu_left /imu_right
```
*Then, by running the script _importData.sh_ we transfer the data inside the bag to two csv files, _Quat_imu_left.csv_ and _Quat_imu_right.csv_. One containing the left wheel imu data and the other one the right wheel imu data, respectively.
*The imu sensor represents angles using quaternions and includes additional data like odomnetry covariance values. To convert the angles we use the script _convert_quat_to_euler.py_. This will create two new files, _imu_left.xlsx_ and _imu_right.xlsx_ which just contains the converted angles, angular velocity and linear acceleration values of the corresponding wheel.
-Note that the python script requires the _xlsxwriter_ module to be able to add data to a new file.