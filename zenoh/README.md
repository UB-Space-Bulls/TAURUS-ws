This directory stores the configurations for the Zenoh routers for each device. The following will be the IP's for each:

| Device | IP | Zenoh router |
|---|---|---|
| Base station laptop | 192.168.1.10 | Connects to the Orin |
| LiteAP radio | 192.168.1.20 | — |
| Bullet AC radio | 192.168.1.21 | — |
| Orin | 192.168.1.50 | Waits for connections |
| ThinkPad | 192.168.1.51 | Connects to the Orin |


This will allow all three devices to see each other's ROS topics. 


The template for this setup file can be found at the following in the docker image:
'/opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5'
