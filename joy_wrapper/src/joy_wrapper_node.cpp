/*
   Remaps joy msgs to joy2 msgs
   joy_wrapper_node.cpp
   Purpose: Allow easier development of code using joystick inputs by having more specific messages
   @author Eric Swanson, Jared Beard
   @version 1.0 7/1/21
 */

#include <rclcpp/rclcpp.hpp>

#include <joy_msgs/msg/joy.hpp>
#include <joy_wrapper_msgs/msg/joy2.hpp>

using std::placeholders::_1;

class JoyWrapper : public rclcpp::Node
{
public:
    MotorInterface(std::string node_name) : Node(node_name){};
    {
      motor_position_publisher_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/motor_positions", 10);

      motor_subscription_ = this->create_subscription<epos_ros2_msgs::msg::MotorCommands>(
      			"/epos_command", 1, std::bind(&MotorInterface::motor_callback, this, _1));
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:

    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr motor_position_publisher_;
    rclcpp::Subscription<epos_ros2_msgs::msg::MotorCommands>::SharedPtr motor_subscription_;

    void motor_callback(const epos_ros2_msgs::msg::MotorCommands::SharedPtr msg)
    {
        motor_commands_ = *msg;
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    void timer_callback(const epos_ros2_msgs::msg::MotorCommands::SharedPtr msg)
    {
        motor_commands_ = *msg;
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

};

int main(int argc, char **argv)
{
    // ROS INITILIZATION ------------------------------------------------------------------------
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<JoyWrapper>("joy_wrapper_node"));

    rclcpp::shutdown();
    return 0;
}
