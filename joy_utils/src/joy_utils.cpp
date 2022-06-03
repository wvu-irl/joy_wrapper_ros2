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
void JoyUtils::joy_callback(const sensor_msgs::msg::Joy::SharedPtr _msg)
{
    prev_msg_ = msg_;
    msg_ = *_msg;

    prev_input_ = input_;
    prev_time_ = time_;
    time_ = msg_.header.stamp;

    for (auto &[key, val] : map_)
        input_[val] = update_button(key);

    deadband_filter();

    toggle_hold = true;
    for (auto &btn : hold_buttons)
        toggle_hold = toggle_hold && input_[map_[btn]].raw && (input_[map_[btn]].change_state > sensitivity_;

    if (toggle_hold == true && ((clock_.now()-toggle_time_).nanoseconds() > sensitivity_))
        hold_on_ = !hold_on_;
        toggle_time_ = clock_.now();

    joy_utils_msgs::msg::JoyUtils temp_msg;
    temp_msg.header = msg_.header;

    temp_msg.A = input_[map_["A"]];
    temp_msg.B = input_[map_["B"]];
    temp_msg.X = input_[map_["X"]];
    temp_msg.Y = input_[map_["Y"]];
    temp_msg.LB = input_[map_["LB"]];
    temp_msg.RB = input_[map_["RB"]];
    temp_msg.back = input_[map_["back"]];
    temp_msg.start = input_[map_["start"]];
    if (controller_ == "Xbox")
        temp_msg.power = input_[map_["power"]];
    temp_msg.LJ = input_[map_["LJ"]];
    temp_msg.RJ = input_[map_["RJ"]];
    temp_msg.LRLJ = input_[map_["LRLJ"]];
    temp_msg.UDLJ = input_[map_["UDLJ"]];
    temp_msg.LRRJ = input_[map_["LRRJ"]];
    temp_msg.UDRJ = input_[map_["UDRJ"]];
    temp_msg.LT = input_[map_["LT"]];
    temp_msg.RT = input_[map_["RT"]];
    temp_msg.LRD = input_[map_["LRD"]];
    temp_msg.UDD = input_[map_["UDD"]];


    pub_.publish(temp_msg;);
}

///
///
///
joy_utils_msgs::msg::Input JoyInput::deadband_filter()
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
joy_utils_msgs::msg::Input JoyInput::update(std::string _input)
{
    joy_utils_msgs::msg::Input btn;

    val = get_value(_input, &msg_);
    prev_val = get_value(_input, &prev_msg_);

    // raw
    btn.raw = val;

    if (prev_val == 0 && val == 1)
    {
        // toggle
        btn.toggle = !input_[map_[_input]].toggle;

        // increment
        btn.increment = input_[map_[_input]].increment++;

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
    if (hold_on_)
    {
        btn.hold = input_[map_[_input]].hold;
    }
    else
    {
        btn.hold = val;
    }

    // time_state
    if (prev_val == val)
    {
        int dt = (msg_.header.stamp - prev_msg_.header.stamp).nanoseconds();
        btn.time_state = input_[map_[_input]].time_state + dt;
    }
    else
    {
        btm.time_state = 0;
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

    // double_click
    if (input_[map_[_input]].rising_edge && prev_input_[map_[_input]].time_state < sensitivity_)
    {
        btn.double_click = true;
    }
    else
    {
        btn.double_click = false;
    }

    return btn;
}

///
///
///
double JoyUtils::get_value(std::string _input, sensor_msgs::msg::Joy *_msg)
{
    if (controller_ == "Xbox")
    {
        switch (_input)
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
        case "back":
            return _msg->buttons[6];
        case "start":
            return _msg->buttons[7];
        case "power":
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
        switch (_input)
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
        case "back":
            return _msg->buttons[8];
        case "start":
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
    this->declare_parameter("hold_buttons", std::vector<std::string>());
    this->declare_parameter("hold_double_click", 1);
    this->declare_parameter("axis_deadband", std::vector<std::string>());
    this->declare_parameter("deadband", std::vector<double>()); //uint8_t
        this->declare_parameter("sensnitivity", 50); //uint8_t

}

///
///
///
void JoyUtils::get_params()
{

    this->get_parameter("controller", controller_)

        this->get_parameter("hold_buttons", hold_buttons_param);
    hold_buttons_ = hold_buttons_param.as_string_array();

    this->get_parameter("hold_double_click", hold_d_click_);

    this->get_parameter("axis_deadband", axis_db_param);
    deadband_axes_ = axis_db_param.as_string_array();

    this->get_parameter("deadband", db_param);
    db_ = db_param.as_double_array();

     this->get_parameter("sensitivity", sensitivity_);
}

///
///
///
JoyUtils::JoyUtils(std::string _node_name)
    : Node(_node_name)
{
    clock_ = rclcpp::Clock();
    prev_time_ = clock_.now();
    time_ = prev_time_;
    hold_on_ = false;

    // Publishers
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joy", 10);

    // Subscriptions
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joy_utils", 1, std::bind(&JoyUtils::joy_callback, this, _1));

    if (controller_ == "Xbox")
    {
        input_val_ = [ "A", "B", "X", "Y", "LB", "RB", "back", "start", "power", "LJ", "RJ", "LRLJ", "UDLJ", "LRRJ", "UDRJ", "LT", "RT", "LRD", "UDD" ];
        msg_.buttons = std::vector<int>(11, 0);
        msg_.axes = std::vector<double>(8, 0);
    }
    else if (controller_ == "Logitech")
    {
        input_val_ = [ "A", "B", "X", "Y", "LB", "RB", "back", "start", "LJ", "RJ", "LRLJ", "UDLJ", "LRRJ", "UDRJ", "LT", "RT", "LRD", "UDD" ];
        msg_.buttons = std::vector<int>(12, 0);
        msg_.axes = std::vector<double>(6, 0);
    }

    joy_utils_msgs::msg::Input temp_button;

    temp_button.raw = 0;
    temp_button.toggle = false;
    temp_button.increment = 0;
    temp_button.hold = 0;
    temp_button.time_state = 0;
    temp_button.rising_edge = false;
    temp_button.falling_edge = false;
    temp_button.double_click = false;

    temp_msg.header.stamp = clock_.now();

    for (int i = 0; i < input_val_.size(); ++i)
    {
        map_.insert({input_val_[i], i});
        prev_input_.push_back(temp_button);
    }

    input_ = prev_input_;

    declare_params();
    get_params();
}

///
///
///
JoyUtils::~JoyUtils()
{
    interface_ptr_->~EPOSWrapper();
}