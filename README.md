# ArduPilot SITL and Simulaiton Interface

Interface between ArduPilot's SITL and custom physics simulations using ROS 2.

<!-- vscode-markdown-toc -->
* 1. [Requirements](#Requirements)
	* 1.1. [Getting Started with ArduPilot SITL](#GettingStartedwithArduPilotSITL)
* 2. [Build](#Build)
* 3. [Content](#Content)
	* 3.1. [Node: sim_interface](#Node:sim_interface)
		* 3.1.1. [Topics](#Topics)
* 4. [Usage](#Usage)

<!-- vscode-markdown-toc-config
	numbering=true
	autoSave=true
	/vscode-markdown-toc-config -->
<!-- /vscode-markdown-toc -->

##  1. <a name='Requirements'></a>Requirements
Tested with:
- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 (Humble Hawksbill)

---

###  1.1. <a name='GettingStartedwithArduPilotSITL'></a>Getting Started with ArduPilot SITL

1. Clone the ArduPilot GitHub repository
    ```
    git clone https://github.com/ArduPilot/ardupilot.git
    ```

2. Run requirements script
    ```
    cd ardupilot
    Tools/environment_install/install-prereqs-ubuntu.sh -y
    . ~/.profile
    ```

3. Relog

4. Build ArduPilot with Rover configuration and the Pixhawk 6X FCU
    ```
    ./waf distclean
    ./waf configure --board Pixhawk6X
    ./waf rover
    ```

5. Run this the first time after build, exit (`Ctrl + C`) when done
    ```
    sim_vehicle.py -w
    ```

6. Start simulation normally with:
    ```
    sim_vehicle.py --console --map
    ```
    This will run ArduPilots own simulation backend and open the simple GCS software [MavProxy](https://ardupilot.org/mavproxy/), but you can connect more GCSs to the sim, like QGroundControl.


##  2. <a name='Build'></a>Build
Clone this package to a ROS 2 workspace and build with colcon:

```
cd ~/ros_ws/src/
git clone git@github.com:Aaveq-Robotics/ardupilot_sitl_sim_interface.git
cd ~/ros_ws/
source /opt/ros/humble/setup.bash
colcon build
```

##  3. <a name='Content'></a>Content 

This package contains [library files](https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON) that interfaces with ArduPilot through UDP, which are files taken from the [ArduPilot Repository](https://github.com/ArduPilot/ardupilot/tree/master), and a node that handles the publishing and subscription of relevant topics.

###  3.1. <a name='Node:sim_interface'></a>Node: sim_interface

####  3.1.1. <a name='Topics'></a>Topics

| Name      |Type   | I/O   |
| ---       | ---   | ---   |
| sim_state | [`aaveq_ros_interfaces::msg::SimState`](https://github.com/Aaveq-Robotics/aaveq_ros_interfaces/blob/main/msg/SimState.msg) | Subscriber |
| control_output | [`aaveq_ros_interfaces::msg::ControlOutput`](https://github.com/Aaveq-Robotics/aaveq_ros_interfaces/blob/main/msg/ControlOutput.msg) | Publisher |

##  4. <a name='Usage'></a>Usage

1. Source workspace and run node:
    ```
    ros2 run ardupilot_sitl_sim_interface sim_interface
    ```

2. Run custom physics engine, for example [usv_sim_2d](https://github.com/Aaveq-Robotics/usv_sim_2d/tree/main)

3. Run ArduPilots SITL in JSON mode to connect custom physics engine, with custom parameter file:

    ```
    sim_vehicle.py -v Rover --map --add-param-file=/home/<user>/<ws>/src/ardupilot_sitl_sim_interface/config/boat.parm
    ```
    > **NOTE:** You can run `sim_vehicle.py` from outside ArduPilot's repository.
    
    > **NOTE:** boat.parm is currently configured as a boat with one thruster and a rudder.

4. In the command terminal (where you ran `sim_vehicle.py`) you can send commands to the simulated FCU
    ```bash
    arm throttle    # Arm vehicle

    rc 3 1900       # Full throttle on RC3 

    rc 3 1100       # Go backwards 

    rc 3 1500       # Stop

    auto            # Set mode to auto, if waypoint mission is avaiable
    ```