##MinhoTeam ROS Package

This repository holds the source code for the ROS package of RoboCup MSL's MinhoTeam. The repository will include all the developed nodes, messages  using ROS, developed in C++.

The package will in future include:

- [x] Hardware Node : Interfaces with hardware, reading information and controlling the actuators.    
- [ ] Localization Node : Provides information about the robot's world model like obstacles, game ball position and self-localization.
- [ ] RTDB Node : Provides information sharing mechanism amongst the other robots, using scheduled UDP scheme (originally by CAMBADA).
- [ ] Artificial Intelligence Node : Provides a data-fusion process where all information is merged into one meaningful piece of information. Also, given the world state/model and the robot's role, provides high level behavioural control.
- [ ] Control Node : Provides low level behavioural control and path planning, using the information provided by AI Node, issuing control commands to HW Node.

*Developed by MinhoTeam @2016*
