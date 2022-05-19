/*
   Joy Utils remapping node
   MotorInterface.hpp
   Purpose: Allow EPOS nodes to communicate with ROS
   @author Jared Beard
   @version 1.0 6/4/21
 */
#ifndef JOY_UTILS_HPP
#define JOY_UTILS_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <joy_utils_msgs/msg/joy_utils.hpp>

#include "joy_input.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct JoyInput
{
	std::string button_;
	double val_;
	double val_prev_;
	int n_;
	rclcpp::Time time_;
	double deadband_;
}

class JoyUtils : public rclcpp::Node
{



public:

	sensor_msgs::msg::Joy input_msg_;
	joy_utils_msgs::msg::JoyUtils remapped_msg_;


	JoyUtils(std::string _node_name);
	~JoyUtils();

private:
	std::string controller_;
	double press_time_ns_;
	std::vector<Button> Buttons_;
	Button hold_;

	rclcpp::Clock clock_;

	rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;

	double get_value(std::string _button, sensor_msgs::msg::Joy &_msg);

	void declare_params();
	void get_params();
};

#endif
