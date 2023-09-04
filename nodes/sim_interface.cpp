#include <array>
#include <chrono>
#include <time.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "aaveq_ros_interfaces/msg/control_output.hpp"
#include "aaveq_ros_interfaces/msg/sim_state.hpp"
#include "ardupilot_sitl_sim_interface/lib_ap_json.hpp"

class SimInterface : public rclcpp::Node
{
public:
    SimInterface() : Node("sim_interface")
    {
        /***** Parameters *****/

        /***** Subscription *****/
        subscriber_sim_state_ = this->create_subscription<aaveq_ros_interfaces::msg::SimState>("sim_state", 1, std::bind(&SimInterface::callback_sim_state, this, std::placeholders::_1));

        /***** Publishers *****/
        publisher_control_output_ = this->create_publisher<aaveq_ros_interfaces::msg::ControlOutput>("control_output", 1);

        /***** Init Variables *****/
        // init the ArduPilot connection
        if (ap_json.InitSockets("127.0.0.1", 9002))
            RCLCPP_INFO_STREAM(get_logger(), "Started socket");
    }

private:
    /***** Variables *****/
    // Node parameters

    // Node variables
    rclcpp::Subscription<aaveq_ros_interfaces::msg::SimState>::SharedPtr subscriber_sim_state_;
    rclcpp::Publisher<aaveq_ros_interfaces::msg::ControlOutput>::SharedPtr publisher_control_output_;

    // Variables
    libAP_JSON ap_json;
    std::array<uint16_t, 16> servo_out;

    /***** Callbacks *****/
    void callback_sim_state(const aaveq_ros_interfaces::msg::SimState::SharedPtr msg)
    {
        if (!ap_json.ap_online)
        {
            RCLCPP_WARN_STREAM(get_logger(), "Recieved simulation data, but ArduPilot is not online");
            return;
        }

        // Send state to ArduPilot
        ap_json.SendState(msg->timestamp,
                          msg->gyro.x, msg->gyro.y, msg->gyro.z,              // gyro
                          msg->accel.x, msg->accel.y, msg->accel.z,           // accel
                          msg->position.x, msg->position.y, msg->position.z,  // position
                          msg->attitude.x, msg->attitude.y, msg->attitude.z,  // attitude
                          msg->velocity.x, msg->velocity.y, msg->velocity.z); // velocity

        // Recieve servo output from ArduPilot controller
        if (ap_json.ReceiveServoPacket(servo_out))
        {
            aaveq_ros_interfaces::msg::ControlOutput msg_servo_out;

            std::copy(servo_out.begin(), servo_out.end(),
                      std::back_inserter(msg_servo_out.servo_pwm));

            publisher_control_output_->publish(msg_servo_out);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimInterface>());
    rclcpp::shutdown();
    return 0;
}
