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
#include <joy_utils_msgs/msg/button.hpp>
#include <joy_utils_msgs/msg/joy_utils.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class JoyUtils : public rclcpp::Node
{

public:

	sensor_msgs::msg::Joy input_msg_, prev_msg_;
	joy_utils_msgs::msg::JoyUtils prev_remapped_msg_, remapped_msg_;

	JoyUtils(std::string _node_name);
	~JoyUtils();

private:
	std::vector<std::string> buttons_, axes_;
	std::string controller_;
	double press_time_ns_;
	std::string hold_button_;
	bool hold_on_;
	int hold_d_click_;
	std::vector<std::string> deadband_axes_;
	std::vector<double> db_;

	rclcpp::Clock clock_;

	rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
	rclcpp::Subscription<joy_utils_msgs::msg::JoyUtils>::SharedPtr sub_;

	double get_value(std::string _button, sensor_msgs::msg::Joy &_msg);

	void declare_params();
	void get_params();

	double raw_update(double _value, bool _hold);
	double increment_update(double _value, bool _hold);
	double time_update(double _value, bool _hold);
	double change_state_update(double _value, bool _hold);
};

float64 raw
bool toggle
int64 increment
float64 hold
float64 time_state
bool rising_edge
bool falling_edge
bool double_click

#endif
	
	rclcpp::Clock *clock_ptr_;

    double update_button(double _value, bool _hold);

	void add_clock(rclcpp::Clock *_clock_ptr) : clock_ptr_(_clock_ptr){};
	void add_deadband(double _db) : deadband_(_db){};
	void add_n(int _n) n_(_n){};
	void add_hold(bool *_hold_ptr) : hold_ptr_(_hold_ptr){ has_hold_ = true};
