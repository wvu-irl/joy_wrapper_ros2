/*
   Interface for Remapping Buttons
   joy_utils.hpp
   Purpose: Allow for convenient remapping of buttons
   @author Jared Beard
   @version 1.0 5/19/22
 */

#include <joy_utils/joy_utils.hpp>

///
///
///
void JoyUtils::joy_callback(const common_interfaces::msg::Joy::SharedPtr _msg)
{
    if (prev_msg_.axes[0] != -2)
    {
        prev_msg_ = input_msg_;
    }
    else
    {
        prev_msg_ = *_msg;
    }
    input_msg_ = *_msg;

    for (auto &b : _msg->buttons)
    {
        
    }
    for (auto &ax : _msg->axes)
    {

    }
    
}

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


///
///
///
double JoyUtils::get_value(std::string _button, sensor_msgs::msg::Joy *_msg)
{
    if (controller_ == "Xbox")
    {
        switch (_button)
        {
        case "A":
            return _msg->buttons[0];
        case "B":
            return _msg->buttons[1];
        case "X":
            return _msg->buttons[2];
        case "Y":
            return _msg->buttons[3];
        case "LB":
            return _msg->buttons[4];
        case "RB":
            return _msg->buttons[5];
        case "Back":
            return _msg->buttons[6];
        case "Start":
            return _msg->buttons[7];
        case "Power":
            return _msg->buttons[8];
        case "LJB":
            return _msg->buttons[9];
        case "RJB":
            return _msg->buttons[10];
        case "LRLJ":
            return _msg->axes[0];
        case "UDLJ":
            return _msg->axes[1];
        case "LRRJ":
            return _msg->axes[2];
        case "UDRJ":
            return _msg->axes[3];
        case "LT":
            return _msg->axes[5];
        case "RT":
            return _msg->axes[4];
        case "LRD":
            return _msg->axes[6];
        case "UDD":
            return _msg->axes[7];
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid Button");
        }
    }
    else if (controller == "Logitech")
    {
        switch (_button)
        {
        case "A":
            return _msg->buttons[1];
        case "B":
            return _msg->buttons[2];
        case "X":
            return _msg->buttons[0];
        case "Y":
            return _msg->buttons[3];
        case "LB":
            return _msg->buttons[4];
        case "RB":
            return _msg->buttons[5];
        case "Back":
            return _msg->buttons[8];
        case "Start":
            return _msg->buttons[9];
        // case "Power":
        //     return _msg->buttons[8];
        case "LJB":
            return _msg->buttons[10];
        case "RJB":
            return _msg->buttons[11];
        case "LRLJ":
            return _msg->axes[0];
        case "UDLJ":
            return _msg->axes[1];
        case "LRRJ":
            return _msg->axes[2];
        case "UDRJ":
            return _msg->axes[3];
        case "LT":
            return _msg->buttons[6];
        case "RT":
            return _msg->buttons[7];
        case "LRD":
            return _msg->axes[4];
        case "UDD":
            return _msg->axes[5];
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid Button");
        }
    }
}

///
///
///
void JoyUtils::declare_params()
{
    // DECLARE PARAMS --------------------------------------------------

    this->declare_parameter("controller", "Logitech");

    this->declare_parameter("press_time", 100); // in nanoseconds

    this->declare_parameter("raw", std::vector<std::string>());
    this->declare_parameter("toggle", std::vector<std::string>());
    this->declare_parameter("increment", std::vector<std::string>());
    this->declare_parameter("increment_n", std::vector<int64_t>());
    this->declare_parameter("axis_hold", std::vector<std::string>());
    this->declare_parameter("hold_button", "back");
    this->declare_parameter("time_on", std::vector<std::string>());
    this->declare_parameter("time_off", std::vector<std::string>());
    this->declare_parameter("change_state", std::vector<std::string>());
}

void JoyUtils::get_params()
{

    this->get_parameter("controller", controller_)

        this->get_parameter("press_time", press_time_ns_); // in nanoseconds

    ///////////////////////////////////////////
    std::vector<Button> Buttons_;

    this->get_parameter("raw", std::vector<std::string>());
    this->get_parameter("toggle", std::vector<std::string>());
    this->get_parameter("increment", std::vector<std::string>());
    this->get_parameter("increment_n", std::vector<int64_t>());
    this->get_parameter("axis_hold", std::vector<std::string>());
    this->declare_parameter("hold_button", "back");
    this->declare_parameter("time_on", std::vector<std::string>());
    this->declare_parameter("time_off", std::vector<std::string>());
    this->get_parameter("change_state", std::vector<std::string>());

    epos2::EPOSParams params;
    int special_param_counter = 0;

    // Motors
    this->get_parameter("motor_names", motor_names_param_);
    params.motor_names = motor_names_param_.as_string_array();
    //++special_param_counter;

    std::vector<int> ids;
    rclcpp::Parameter motor_ids_param = this->get_parameter("motor_ids");
    ids = std::vector<int>(motor_ids_param.as_integer_array().begin(),
                           motor_ids_param.as_integer_array().end());
    //++special_param_counter;

    for (std::vector<int>::size_type i = 0; i < ids.size(); ++i)
    {
        params.motor_ids.insert(std::make_pair(params.motor_names[i], ids[i]));
        params.motor_inds.insert(std::make_pair(params.motor_names[i], i));
    }

    // Maxon Motors
    epos2::MaxonMotor temp_motor;
    for (std::vector<int>::size_type i = 0; i < ids.size(); ++i)
    {
        temp_motor.index = ids[i];
        this->get_parameter("motor_params/gear_ratio", temp_motor.gear_ratio);
        this->get_parameter("motor_params/counts_per_rev", temp_motor.counts_per_rev);
        this->get_parameter("motor_params/wheel_radius/cm", temp_motor.wheel_radius);
        this->get_parameter("motor_params/kT", temp_motor.kT);
        this->get_parameter("motor_params/user_limits/ang_vel_rpm", temp_motor.ang_vel_limit);
        this->get_parameter("motor_params/user_limits/acc_rpm", temp_motor.acc_limit);
        this->get_parameter("motor_params/absolute_limits/curr_stall_a", temp_motor.stall_current);
        this->get_parameter("motor_params/user_limits/curr_inst_a", temp_motor.instantaneous_current_limit);
        this->get_parameter("motor_params/user_limits/curr_cont_a", temp_motor.continuous_current_limit);

        params.motors.push_back(temp_motor);
    }

    // EPOS Modules
    this->get_parameter("epos_module/protocol", params.protocol_stack_name);
    this->get_parameter("epos_module/com_interface", params.interface_name);
    this->get_parameter("epos_module/port", params.port_name);
    this->get_parameter("epos_module/baud_rate", params.baud_rate);
    this->get_parameter("motor_close_timeout", params.motor_close_timeout);

    // // Logging
    this->get_parameter("logging/throttle", params.throttle);

    return params;
}

///
///
///
JoyUtils::JoyUtils(std::string _node_name) : Node(_node_name, "ftr")
{
    // PARAM INITILIZATION ------------------------------------------------------------------------
    declare_params();
    get_params();
    this->clock_ = rclcpp::Clock();

    // Publishers
    this->pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joy", 10);

    // Subscriptions
    this->sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joy_utils", 1, std::bind(&JoyUtils::joy_callback, this, _1));
}

///
///
///
JoyUtils::~JoyUtils()
{
    interface_ptr_->~EPOSWrapper();
}