Steps:

To run sim
ros2 launch rover_description gazebo.launch.py

Then use 
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
this connect keyboard inputs to the sim

Open a new terminal and open the docker session:
docker exec -it rover-dev-container bash

In new terminal run the foxglove bridge to be able to connect
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

Open foxglove desktop app, click Open Connection. 
This will allow you to connect to the websocket that is open for the foxglove bridge. In this case, connect to:
ws://localhost:8765

Once connected, import the panel layout from the repo. It is fonud in DEIMOS-ws/src/foxglove/DEIMOST-Panel.json.

Once connected, you need to go to your profile, then settings, then Desktop, then set ROS_PACKAGE_PATH to the path that the DEIMOS-ws is at on your device.

