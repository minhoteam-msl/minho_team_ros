# hardware_node

This node is one of the base nodes in minho-team-ros package, available on team's GitHub for free use. This node allows communication with the centralized hardware controller present in the robot.

This node subscribes to the following messages:

* *controlInfo* - Allows to send commands to interact with the platform
* *teleop* - Allows to enable or disable teleoperation for the platform

This node published the following messages:

* *hardwareInfo* - Publishes information that is read from the platform, including:
	
	* Odometry (encoders)
	* Batteries
	* IMU (orientation)
	* Ball detection sensor
	* Free-wheel operation
	
TODO : Re-write I2C libraries to run the node directly using a RaspberryPi as the host controller for the hardware.
