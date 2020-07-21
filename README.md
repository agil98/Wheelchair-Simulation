# Wheelchair-Simulation

## Publishing angular velocity:

1. Edit the `run_and_record.launch` file, located [here](torque_controller/launch) in *torque_controller/launch*. Edit the `<arg>` tags at the top of the file to contain the correct directories for saving csvs and plots. **Note: tilde operator does not work for plots file path. Use absolute paths.**

2. Open Gazebo with `roslaunch wheelchair velcontroller.launch`. This will spawn the wheelchair.

3. Run `roslaunch torque_controller run_and_record.launch`. Choose the experimental csv file in the popup dialogue window. You can also run `roslaunch torque_controller run_and_record.launch fname:="your_file_name"` to specify a file name for saved csv files. File name defaults to *test_file*.

4. Plots should appear at the end of the simulation. You can save these manually with the save button in the popup window. When finished, close the popup window to terminate the node.

**Note: at the moment, imu_sensor plugins seem to break when resetting the simulation within Gazebo. To run another trial, terminate the Gazebo node and repeat from step 2.**
