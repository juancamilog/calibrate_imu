# calibrate_imu
This  ROS node will read messages published to the /imu/data and /imu/mag topics. For the moment it only performs magnetometer calibration. 


To run the node

```shell
rosrun calibrate_imu calibrate_imu.py
```

To start collecting samples from the IMU

```shell
rosservice call /start_sampling
```

To calibrate the magnetometer using the current data samples
```shell
rosservice call /calibrate_mag
```

<!-- This will store the calibration parameters in a file called "mag_calibration_d_m_y_H_M_S", where d_m_y_H_M_S is the time the file was created.  For an example on how to read the file you can look at the plot_calibration_data.py script

To visualize the raw data from the magnetometer and the calibrated data

```shell
./plot_calibration_data.py mag_calibration_d_m_y_H_M_S
``` -->

If you have any questions or comments, please shoot me at email at gamboa at cim dot mcgill dot ca

# Limitations
This branch does not save the resulting configuration, it is only printed to the terminal, you will need to perform the adjustment in another node such as the `imu_filter_madwick` from the `imu_tools` package. 