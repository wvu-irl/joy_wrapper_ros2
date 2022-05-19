/*
   Button object to do operations on
   joy_input.cpp
   Purpose: simulates button/axis for manipulating in util
   @author Jared Beard
   @version 1.0 6/4/21
 */

#include <joy_utils/joy_input.hpp>

///
///
///
double JoyInput::raw_update(double _value, bool _hold)
{
    if (!_hold)
    {
        val_prev_ = val_;
        val_ = _value;
    }
    return val_;
}

///
///
///
double JoyInput::increment_update(double _value, bool _hold)
{
}

///
///
///
double JoyInput::time_on_update(double _value, bool _hold)
{
}

///
///
///
double JoyInput::time_off_update(double _value, bool _hold)
{
}

///
///
///
double JoyInput::change_state_update(double _value, bool _hold)
{
}

///
///
///

double JoyInput::update_button(double _value)
{
    if (!has_hold_)
    {
        _hold = false;
    }
    else
    {
        _hold = *hold_ptr;
    }

    switch (_button)
    {
    case "raw":
        return raw_update(_value, _hold);
    case "toggle":
        return increment_update(_value, _hold);
    case "increment":
        return increment_update(_value, _hold);
    case "time_on":
        return raw_update(_value, _hold);
    case "time_off":
        return raw_update(_value, _hold);
    case "change_state":
        return raw_update(_value, _hold);
    default:
        RCLCPP_WARN(this->get_logger(), "Invalid Mode");
    }
}

///
///
///
JoyInput::JoyInput(std::string _button, std::string _mode)
    : button_(_button), mode_(_mode)
{
    val_ = 0;
    val_prev_ = 0;
    if (_mode == "toggle")
        n_ = 1;
    has_hold_ = false;
}
