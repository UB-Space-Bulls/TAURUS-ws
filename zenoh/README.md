This directory stores the configurations for the Zenoh routers for each device. The following will be the IP's for each:

Basestation laptop: 192.168.1.10
LiteAP Radio: 192.168.1.20
Bullet AC Radio: 192.168.1.21
Orin: 192.168.1.50
Thinkpad: 192.168.1.51

The Zenoh routers will be set to do the following:

Base: Connect to orin
Orin: Wait for connections
Thinkpad: Connect to orin


This will allow all three devices to see each other's ROS topics. 


