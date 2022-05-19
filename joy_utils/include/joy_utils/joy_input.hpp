/*
   Button object to do operations on
   joy_input.hpp
   Purpose: simulates button/axis for manipulating in util
   @author Jared Beard
   @version 1.0 6/4/21
 */
#ifndef JOY_INPUT_HPP
#define JOY_INPUT_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

class JoyInput
{
public:

	bool *hold_ptr_;
	bool has_hold_;
	
	rclcpp::Clock *clock_ptr_;

    double update_button(double _value, bool _hold);

	void add_clock(rclcpp::Clock *_clock_ptr) : clock_ptr_(_clock_ptr){};
	void add_deadband(double _db) : deadband_(_db){};
	void add_n(int _n) n_(_n){};
	void add_hold(bool *_hold_ptr) : hold_ptr_(_hold_ptr){ has_hold_ = true};

	JoyInput(std::string _button, std::string _mode, double _db);
	~JoyInput(){};

private:

	double raw_update(double _value, bool _hold);
	double increment_update(double _value, bool _hold);
	double time_on_update(double _value, bool _hold);
	double time_off_update(double _value, bool _hold);
	double change_state_update(double _value, bool _hold);
};

#endif
