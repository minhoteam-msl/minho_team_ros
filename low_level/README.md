# minho_team_lowlevel
### Full ROS Support
This node, although running in Arduino, has full ROS Support using rosserial and ros_lib. To run the node, one has to plugin in the arduino box and run $ rosrun rosserial_pyhton serial_node.py /dev/ttyACM*

* Subscribers:
	  - [x] controlInfo - Send control commands to platform 
	  - [x] teleop - Enable/Disbale teleoperation
* Publishers:
  	- [x] hardwareInfo - Send info about platform's peripherials
* Services:
	- [x] requestResetEncoders - Reset the value of all 3 encoders
	- [x] requestResetIMU - Reset IMU's 0º reference value
	- [x] requestSetOmniProps - Set OMNI 3MD's PID and RAMP 
		* Send is_set = true to set, is_set = false to get current config on response.
		* Send omniConf.* = >0 to change a property. omniConf.x = 0 wont change.
		* Current configuration will be returned in any case.
	- [x] requestIMULinTable- Set IMU linearization table
		* Send is_set = true to set, is_set = false to get current config on response. 
		* Send imu_values as "[val1,val2...]"
		* Current configuration will be returned in any case.
### Baudrate 57600
### Timeout 200ms

##Installation
* $ sudo chmod 777 install
* $ ./install

##Upload to board
* $ sudo chmod 777 upload
* $ ./upload
