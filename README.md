# ArduPilot SITL and Simulaiton Interface

Interface between ArduPilot's SITL and custom physics simulations using ROS 2.

## Content 

This package contains [library files](https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON) that interfaces with ArduPilot through UDP, which are files taken from the [ArduPilot Repository](https://github.com/ArduPilot/ardupilot/tree/master), and a node that handles the publishing and subscription of relevant topics.

### Node: sim_interface

#### Topics

| Name      |Type   | I/O   |
| ---       | ---   | ---   |
| sim_state | [`aaveq_ros_interfaces::msg::SimState`](https://github.com/Aaveq-Robotics/aaveq_ros_interfaces/blob/main/msg/SimState.msg) | Subscriber |
| control_output | [`aaveq_ros_interfaces::msg::ControlOutput`](https://github.com/Aaveq-Robotics/aaveq_ros_interfaces/blob/main/msg/ControlOutput.msg) | Publisher |

## Usage

1. Run node:
    ```
     ros2 run ardupilot_sitl_sim_interface sim_interface
    ```

2. Run custom physics engine.

3. Run ArduPilots SITL in JSON mode:

    ```
    sim_vehicle.py -v Rover -f JSON --map
    ```
    Alternatively load with custom parameter file:

    ```
    sim_vehicle.py -v Rover -f JSON --map --add-param-file=<name>.parm
    ```