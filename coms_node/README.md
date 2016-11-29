## coms_node

This node allow the robot to send it's information to a Multicast group, using UDP, where the other robots and the
basestation are listening. In this manner, wbery agent shares its information. The node then creates a publisher for 
each agent in its internal ROS network, making information avaliable for any possible subscriber.

- [x] Allows Multicast UDP communication
- [x] Implementation using Multi-threading in data packet reading and processing
- [x] Serialization and deserialization of ROS Messages
- [x] For real and simulated Robots
- [x] Shares interAgentInfo messages (containing goalKeeperInfo+hardwareInfo)
- [x] Information realyed inside ROS network with agent-related topics

Node Usage:
* Real robot : rosrun minho\_team\_ros coms_node
* Simulated robot : rosrun minho\_team\_ros coms_node -s <\TARGET_ROBOT\>

<\TARGET_ROBOT\> ranges from 1 to 5 in simulated robots.

*Developed by MinhoTeam @2016*
