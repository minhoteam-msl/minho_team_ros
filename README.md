##MinhoTeam ROS Package

This repository holds the source code for the ROS package of RoboCup MSL's MinhoTeam. The repository will include all the developed nodes, messages  using ROS, developed in C++.

The package will in future include:

- [x] localization_node : Provides information about the robot's world model like obstacles, game ball position and self-localization.
- [x] coms_node : Provides information sharing mechanism amongst the other robots, using Multicast UDP communication.
- [ ] ai_node : Provides a data-fusion process where all information is merged into one meaningful piece of information. Also, given the world state/model and the robot's role, provides high level behavioural control.
- [ ] control_node : Provides low level behavioural control and path planning, using the information provided by AI Node, issuing control commands to HW Node.
- [x] hardware_node : Provides control and feedback over robot's hardware using arduino. This node requires rosserial_python serial_node.py node to be executed, in order to make the bridge between serial and network.

Please refer to each node's README.md for further information.

*Developed by MinhoTeam @2016*
