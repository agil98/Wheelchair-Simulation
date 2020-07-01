## Recording IMU data

* First, we have run the _wheelchair_data_ package which subscribes to the Imu topic, converts the angles and calculates linear velocities, and then publishes it into a new topic.
``` 
rosrun wheelchair_data imu_to_wheelchair_data.py
```
* Then, the data published by the topic can be recorded into a rosbagb called _imu_ using the following command

```
rosbag record -O imu /imu_data_left /imu_data_right /imu_data_frame
```

* Lastly, by running the script _rosbag_to_xlsx.sh_ we transfer the data inside the bag to three xlsx files containing the data for the frame and the two wheels.

### Alternative Method

The imu data can be recorded directly from the topic published by the sensors and then transformed into a readable spreedsheet by using the shell script mentioned above. Notice that although the topics have differnt names as the ones mentioned before, all their data will be recorded to same location, i.e. imu.bag 
```
rosbag record -O imu /imu_sensor_left_ /imu_sensor_right /imu_sensor_frame
```
This will create three excel files just like mentioned above, however, the files will not contain linear velocities and the orientation angles will be in quaternions. To convert the angles, we can use the script _quat_to_euler.py_. This will create three new files containing the converted angles, angular velocity and linear acceleration values of the corresponding wheel.
-Note that the python script requires the _xlsxwriter_ module to be able to add data to a new file.