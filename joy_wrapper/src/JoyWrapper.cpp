/*
   Interface for Remapping Buttons
   joy_wrapper.cpp
   Purpose: Allow for convenient remapping of buttons
   @author Jared Beard
   @version 1.0 5/19/22
 */
#include <iostream>

#include <joy_wrapper/JoyWrapper.hpp>

///
///
///
void JoyWrapper::joy_callback(const sensor_msgs::msg::Joy::SharedPtr _msg)
{
    prev_msg_ = msg_;
    msg_ = *_msg;

    prev_input_ = input_;
    prev_time_ = time_;
    time_ = msg_.header.stamp;

    for (auto &m : map_)
        input_[m.second] = update(m.first);

    deadband_filter();

    // bool toggle_hold = true;
    // for (auto &btn : hold_buttons_)
    //     toggle_hold = toggle_hold && input_[map_[btn]].raw; // && (input_[map_[btn]].time_state > sensitivity_);

    // if (toggle_hold == true && ((clock_.now()-toggle_time_).nanoseconds() > 5000*sensitivity_))
    // {
    //     hold_on_ = !hold_on_;
    //     toggle_time_ = clock_.now();
    // }
    joy_wrapper_msgs::msg::JoyWrapper temp_msg;
    temp_msg.header = msg_.header;

    temp_msg.a = input_[map_["A"]];
    temp_msg.b = input_[map_["B"]];
    temp_msg.x = input_[map_["X"]];
    temp_msg.y = input_[map_["Y"]];
    temp_msg.lb = input_[map_["LB"]];
    temp_msg.rb = input_[map_["RB"]];
    temp_msg.back = input_[map_["back"]];
    temp_msg.start = input_[map_["start"]];
    if (controller_ == "Xbox")
        temp_msg.power = input_[map_["power"]];
    temp_msg.lj = input_[map_["LJ"]];
    temp_msg.rj = input_[map_["RJ"]];
    temp_msg.lrlj = input_[map_["LRLJ"]];
    temp_msg.udlj = input_[map_["UDLJ"]];
    temp_msg.lrrj = input_[map_["LRRJ"]];
    temp_msg.udrj = input_[map_["UDRJ"]];
    temp_msg.lt = input_[map_["LT"]];
    temp_msg.rt = input_[map_["RT"]];
    temp_msg.lrd = input_[map_["LRD"]];
    temp_msg.udd = input_[map_["UDD"]];

    temp_msg.hold_on = hold_on_;
    pub_->publish(temp_msg);
}

///
///
///
void JoyWrapper::deadband_filter()
{
    for (auto& axis: deadband_axes_)
    {
        if (fabs(input_[map_[axis]].raw) <= db_[map_[axis]])
            input_[map_[axis]].raw = 0;
    }
}

///
///
///
joy_wrapper_msgs::msg::Input JoyWrapper::update(std::string _input)
{
    joy_wrapper_msgs::msg::Input btn;

    double val = get_value(_input, &msg_);
    double prev_val = get_value(_input, &prev_msg_);

    // raw
    btn.raw = val;

    if (prev_val == 0 && val == 1)
    {
        // toggle
        btn.toggle = !input_[map_[_input]].toggle;

        // increment
        // std::cout << input_[map_[_input]].increment << std::endl;
        // std::cout << input_[map_[_input]].increment++ << std::endl;

        btn.increment = ++(input_[map_[_input]].increment);
        // std::cout << btn.increment << std::endl;
        // rising_edge
        btn.rising_edge = true;
    }
    else
    {
        // toggle
        btn.toggle = input_[map_[_input]].toggle;

        // increment
        btn.increment = input_[map_[_input]].increment;

        // rising_edge
        btn.rising_edge = false;
    }

    // hold
    // if (hold_on_)
    // {
    //     btn.hold = input_[map_[_input]].hold;
    // }
    // else
    // {
    //     btn.hold = val;
    // }

    // time_state
    if (prev_val == val)
    {
        rclcpp::Time t1 = msg_.header.stamp;
        rclcpp::Time t2 = prev_msg_.header.stamp;
        int dt = (t1 - t2).nanoseconds();
        btn.time_state = input_[map_[_input]].time_state + dt;
    }
    else
    {
        btn.time_state = 0;
    }

    // falling_edge
    if (prev_val == 1 && val == 0)
    {
        btn.falling_edge = true;
    }
    else
    {
        btn.falling_edge = false;
    }

    // // double_click
    // if (input_[map_[_input]].rising_edge && prev_input_[map_[_input]].time_state < sensitivity_)
    // {
    //     btn.double_click = true;
    // }
    // else
    // {
    //     btn.double_click = false;
    // }

    return btn;
}

///
///
///
double JoyWrapper::get_value(std::string _input, sensor_msgs::msg::Joy *_msg)
{
    if (controller_ == "Xbox")
    {
        if (_input == "A")
            return _msg->buttons[0];
        if (_input == "B")
            return _msg->buttons[1];
        if (_input == "X")
            return _msg->buttons[2];
        if (_input == "Y")
            return _msg->buttons[3];
        if (_input == "LB")
            return _msg->buttons[4];
        if (_input == "RB")
            return _msg->buttons[5];
        if (_input == "back")
            return _msg->buttons[6];
        if (_input == "start")
            return _msg->buttons[7];
        if (_input == "power")
            return _msg->buttons[8];
        if (_input == "LJB")
            return _msg->buttons[9];
        if (_input == "RJB")
            return _msg->buttons[10];
        if (_input == "LRLJ")
            return _msg->axes[0];
        if (_input == "UDLJ")
            return _msg->axes[1];
        if (_input == "LRRJ")
            return _msg->axes[2];
        if (_input == "UDRJ")
            return _msg->axes[3];
        if (_input == "LT")
            return _msg->axes[5];
        if (_input == "RT")
            return _msg->axes[4];
        if (_input == "LRD")
            return _msg->axes[6];
        if (_input == "UDD")
            return _msg->axes[7];
    }
    else if (controller_ == "Logitech")
    {
        if (_input == "A")
            return _msg->buttons[1];
        if (_input == "B")
            return _msg->buttons[2];
        if (_input == "X")
            return _msg->buttons[0];
        if (_input == "Y")
            return _msg->buttons[3];
        if (_input == "LB")
            return _msg->buttons[4];
        if (_input == "RB")
            return _msg->buttons[5];
        if (_input == "back")
            return _msg->buttons[8];
        if (_input == "start")
            return _msg->buttons[9];
        // if (_input == "Power")
        //     return _msg->buttons[8];
        if (_input == "LJB")
            return _msg->buttons[10];
        if (_input == "RJB")
            return _msg->buttons[11];
        if (_input == "LRLJ")
            return _msg->axes[0];
        if (_input == "UDLJ")
            return _msg->axes[1];
        if (_input == "LRRJ")
            return _msg->axes[2];
        if (_input == "UDRJ")
            return _msg->axes[3];
        if (_input == "LT")
            return _msg->buttons[6];
        if (_input == "RT")
            return _msg->buttons[7];
        if (_input == "LRD")
            return _msg->axes[4];
        if (_input == "UDD")
            return _msg->axes[5];
    }
}

///
///
///
void JoyWrapper::declare_params()
{
    // DECLARE PARAMS --------------------------------------------------

    this->declare_parameter("controller", "Logitech");
    this->declare_parameter("hold_buttons", std::vector<std::string>());
    this->declare_parameter("hold_double_click", 1);
    this->declare_parameter("axis_deadband", std::vector<std::string>());
    this->declare_parameter("deadband", std::vector<double>()); //uint8_t
    this->declare_parameter("sensitivity", 50); //uint8_t

}

///
///
///
void JoyWrapper::get_params()
{
    rclcpp::Parameter hold_buttons_param, axis_db_param, db_param;


    this->get_parameter("controller", controller_);
    RCLCPP_WARN(this->get_logger(), ("Controller: " + controller_).c_str());

    this->get_parameter("hold_buttons", hold_buttons_param);
    hold_buttons_ = std::vector<std::string>(hold_buttons_param.as_string_array().begin(), hold_buttons_param.as_string_array().end());

    RCLCPP_WARN(this->get_logger(), "Hold buttons: ");
    std::cout << hold_buttons_.size() << std::endl;
    for (auto &btn : hold_buttons_)
        RCLCPP_WARN(this->get_logger(), btn.c_str());

    this->get_parameter("hold_double_click", hold_d_click_);

    RCLCPP_WARN(this->get_logger(), ("Double click: " + std::to_string(hold_d_click_)).c_str());

    this->get_parameter("axis_deadband", axis_db_param);
    deadband_axes_ = axis_db_param.as_string_array();

    RCLCPP_WARN(this->get_logger(), "Deadband Axes: ");
    std::cout << deadband_axes_.size() << std::endl;
    for (auto &ax : deadband_axes_)
        RCLCPP_WARN(this->get_logger(), ax.c_str());

    this->get_parameter("deadband", db_param);
    db_ = std::vector<double>(db_param.as_double_array().begin(),db_param.as_double_array().end()) ;

    RCLCPP_WARN(this->get_logger(), "Deadband: ");
    std::cout << db_.size() << std::endl;
    for (auto &db : db_)
        RCLCPP_WARN(this->get_logger(), std::to_string(db).c_str());

    this->get_parameter("sensitivity", sensitivity_);

    RCLCPP_WARN(this->get_logger(), ("Sensitivity: " + std::to_string(sensitivity_)).c_str());

}

///
///
///
JoyWrapper::JoyWrapper(std::string _node_name)
    : Node(_node_name)
{
    clock_ = rclcpp::Clock();
    prev_time_ = clock_.now();
    time_ = prev_time_;
    hold_on_ = false;

    declare_params();
    get_params();

    // Publishers
    pub_ = this->create_publisher<joy_wrapper_msgs::msg::JoyWrapper>("/joy_wrapper", 1);

    // Subscriptions
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&JoyWrapper::joy_callback, this, _1));

    if (controller_ == "Xbox")
    {
        input_val_ = { "A", "B", "X", "Y", "LB", "RB", "back", "start", "power", "LJ", "RJ", "LRLJ", "UDLJ", "LRRJ", "UDRJ", "LT", "RT", "LRD", "UDD" };
        // std::vector<double> temp(11,0);
        msg_.buttons = std::vector<int>(11, 0);
        for (int i = 0; i < 8; ++i)
            msg_.axes.push_back(0.0);
    }
    else if (controller_ == "Logitech")
    {
        input_val_ = { "A", "B", "X", "Y", "LB", "RB", "back", "start", "LJ", "RJ", "LRLJ", "UDLJ", "LRRJ", "UDRJ", "LT", "RT", "LRD", "UDD" };
        msg_.buttons = std::vector<int>(12, 0);
        for (int i = 0; i < 6; ++i)
            msg_.axes.push_back(0.0);
    }

    joy_wrapper_msgs::msg::Input temp_button;

    temp_button.raw = 0;
    temp_button.toggle = false;
    temp_button.increment = 0;
    temp_button.hold = 0;
    temp_button.time_state = 0;
    temp_button.rising_edge = false;
    temp_button.falling_edge = false;
    temp_button.double_click = false;

    prev_time_ = clock_.now();
    time_ = prev_time_;
    toggle_time_ = prev_time_;

    for (int i = 0; i < input_val_.size(); ++i)
    {
        map_.insert({input_val_[i], i});
        prev_input_.push_back(temp_button);
    }

    input_ = prev_input_;

}
