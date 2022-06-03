/*
   Remaps joy msgs to joy2 msgs
   joy_wrapper_node.cpp
   Purpose: Allow easier development of code using joystick inputs by having more specific messages
   @author Jared Beard
   @version 1.0 5/1/22
 */
#include <rclcpp/rclcpp.hpp>

#include <joy_wrapper/JoyWrapper.hpp>

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyWrapper>("joy_wrapper_node"));
    rclcpp::shutdown();

    return 0;
}
