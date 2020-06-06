## Recording IMU data into a csv file
We want to be able to record into a csv file the data provided by the imu sensor.

### Recording the mesaages published by the imu sensor
We use the following command to record the messages published by the imu sensors into a file called `imu.bag`.
```
rosbag record -O imu /imu_left /imu_right
```
### Importing .bag to .csv
By running `importData.sh` we transfer the data inside the bag to two csv files, one containing the left wheel imu data and the other one the right wheel imu data.