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
#include <math.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <joy_utils_msgs/msg/input.hpp>
#include <joy_utils_msgs/msg/joy_utils.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class JoyUtils : public rclcpp::Node
{

public:
	

	JoyUtils(std::string _node_name);
	~JoyUtils(){};

private:

	std::map<std::string, int> map_;
	sensor_msgs::msg::Joy msg_, prev_msg_;
	rclcpp::Time prev_time_, time_, toggle_time_;
	std::vector<joy_utils_msgs::msg::Input> prev_input_, input_;
	std::string controller_;
	std::vector<std::string> input_val_, hold_buttons_;
	bool hold_on_;
	int hold_d_click_;
	std::vector<std::string> deadband_axes_;
	std::vector<double> db_;
	double sensitivity_;
	
	rclcpp::Clock clock_;

	rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
	rclcpp::Subscription<joy_utils_msgs::msg::JoyUtils>::SharedPtr sub_;

    

	

double update_button(double _value, bool _hold);
	double get_value(std::string _input, sensor_msgs::msg::Joy *_msg);
	joy_utils_msgs::msg::Input update(std::string _input);
	double deadband_filter();

	void declare_params();
	void get_params();

};

#endif
	
