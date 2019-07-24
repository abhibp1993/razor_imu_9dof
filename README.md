ROS2 Drivers for Sparkfun Razor IMU 9DOF
--------------------------
[Under Development]

This package adapts ROS package "razor_imu_9dof" (http://wiki.ros.org/razor_imu_9dof) to ROS2. 
Presently the following features are implemented: 

* ROS2 Node: `imu_node` publishes the orientation, linear acceleration and angular velocity on topic `imu`.  
 

Install Arduino firmware
-------------------------
1) For SEN-14001 (9DoF Razor IMU M0), you will need to follow the same instructions as for the default firmware on https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide and use an updated version of SparkFun_MPU-9250-DMP_Arduino_Library from https://github.com/lebarsfa/SparkFun_MPU-9250-DMP_Arduino_Library (an updated version of the default firmware is also available on https://github.com/lebarsfa/9DOF_Razor_IMU).

2) Open ``src/Razor_AHRS/Razor_AHRS.ino`` in Arduino IDE. Note: this is a modified version
of Peter Bartz' original Arduino code (see https://github.com/ptrbrtz/razor-9dof-ahrs). 
Use this version - it emits linear acceleration and angular velocity data required by the ROS Imu message

3) Select your hardware here by uncommenting the right line in ``src/Razor_AHRS/Razor_AHRS.ino``, e.g.

<pre>
// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware here by uncommenting one line!
//#define HW__VERSION_CODE 10125 // SparkFun "9DOF Razor IMU" version "SEN-10125" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
#define HW__VERSION_CODE 14001 // SparkFun "9DoF Razor IMU M0" version "SEN-14001"
//#define HW__VERSION_CODE 10183 // SparkFun "9DOF Sensor Stick" version "SEN-10183" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10321 // SparkFun "9DOF Sensor Stick" version "SEN-10321" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)
</pre>

4) Upload Arduino sketch to the Sparkfun 9DOF Razor IMU board


Install and Configure ROS Package
---------------------------------
1) Download code:

	```shell script
    cd ~/ros2_wc/src
    git clone https://github.com/abhibp1993/razor_imu_9dof.git
    ```   
 
2) Build the package:
    
   On Linux: 
   ```shell script
   cd ..
   colcon build --symlink-install
   ```
   
   On Windows: 
   ```shell script
   cd ..
   colcon build --merge-install
   ```
	
3) Run the node:
    
   On Linux:
   ```shell script
   source install/setup.bash
   ros2 run razor_imu_9dof imu_node 
   ```
   
   On Windows:
   ```shell script
   call "install/setup.bat"
   ros2 run razor_imu_9dof imu_node 
   ```


Calibrate
---------
For best accuracy, follow the tutorial to calibrate the sensors:

http://wiki.ros.org/razor_imu_9dof

An updated version of Peter Bartz's magnetometer calibration scripts from https://github.com/ptrbrtz/razor-9dof-ahrs is provided in the ``magnetometer_calibration`` directory.

Update ``my_razor.yaml`` with the new calibration parameters.
