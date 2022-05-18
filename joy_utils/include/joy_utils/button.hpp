/*
   Button object to do operations on
   MotorInterface.hpp
   Purpose: Allow EPOS nodes to communicate with ROS
   @author Jared Beard
   @version 1.0 6/4/21
 */
#ifndef JOY_BUTTON_HPP
#define JOY_BUTTON_HPP

#include <string>

class Button
{
public:

	std::string button_;
    std::string type_;
    std::string index_;

    


	Button(std::string _node_name);
	~Button();

private:
	std::string controller_;
	double press_time_;
	std::vector<std::string> raw_, toggle_, increment_, axis_hold_, timer_, change_state_;
	std::vector<int> inrement_n_;
    raw: ['Y']                        # if unspecified, will default to raw
    toggle: ['A', 'B']
    increment: ['Y']
    increment_n: ['5']
    axis_hold: ['LRLJ', 'UDLJ']
    timer: ['None']
    change: ['None']

	rclcpp::TimerBase::SharedPtr status_timer_, fault_timer_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_state_publisher_;

	// rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr error_monitor_publisher_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_command_subscription_;

	void motor_callback(const sensor_msgs::msg::JointState::SharedPtr _msg);
	bool position_status = false, velocity_status = false, effort_status = false;
	bool effort_as_current = false;
	void status_callback();
	void fault_callback();

	void drive_motors(sensor_msgs::msg::JointState &_msg);

	std::vector<rclcpp::Parameter> special_params_;
	rclcpp::Parameter motor_names_param_;
	void declare_params();
	epos2::EPOSParams get_params();
};

#endif
