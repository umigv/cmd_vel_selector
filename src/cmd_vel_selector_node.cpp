#include "cmd_vel_selector_node.h"

#include <cstdint>
#include <limits>

namespace umigv {
namespace cmd_vel_selector {

void CmdVelSelectorNode::add_teleop(
    const geometry_msgs::Twist::ConstPtr &cmd_vel_ptr
) {
    ROS_DEBUG_STREAM("received a teleoperated velocity command");
    selector_.accept_teleop(*cmd_vel_ptr);
}

void CmdVelSelectorNode::add_autonomous(
    const geometry_msgs::Twist::ConstPtr &cmd_vel_ptr
) {
    ROS_DEBUG_STREAM("received an autonomous velocity command");
    selector_.accept_autonomous(*cmd_vel_ptr);
}

void CmdVelSelectorNode::check_button(
    const sensor_msgs::Joy::ConstPtr &joy_ptr
) {
    if (joy_button_ >= joy_ptr->buttons.size()) {
        ROS_ERROR_STREAM(
            "CmdVelSelectorNode::check_button: invalid button " << joy_button_
        );

        return;
    }

    const std::int32_t data = joy_ptr->buttons[joy_button_];

    ROS_DEBUG_STREAM("received a sensor_msgs/Joy message with button data "
                     << data);

    const auto next_button_state = get_next_state(data);

    if (next_button_state == ButtonState::Pressed
        && button_state_ == ButtonState::Unpressed) {
        switch (flip_mode()) {
        case OutputMode::Teleoperated: {
            ROS_INFO_STREAM("switching to teleoperated mode");

            break;
        } case OutputMode::Autonomous: {
            ROS_INFO_STREAM("switching to autonomous mode");

            break;
        }
        }
    }

    button_state_ = next_button_state;
}

void CmdVelSelectorNode::poll_command(const ros::TimerEvent&) {
    const auto maybe_cmd_vel = selector_.latest();

    if (!maybe_cmd_vel) {
        ROS_WARN_STREAM_THROTTLE(1.0, "no new command velocity at poll time");

        return;
    }

    cmd_vel_publisher_.publish(maybe_cmd_vel.value());
}

ros::Subscriber CmdVelSelectorNode::make_teleop_subscriber() {
    using MessageT = geometry_msgs::Twist;

    static constexpr char TOPIC[] = "teleop/cmd_vel";
    static constexpr std::uint32_t QUEUE_SIZE = 10;
    static constexpr auto &&CALLBACK_PTR = &CmdVelSelectorNode::add_teleop;

    auto &&subscriber = node_.subscribe<MessageT>(TOPIC, QUEUE_SIZE,
                                                  CALLBACK_PTR, this);

    ROS_DEBUG_STREAM("subscribed to teleop cmd_vel on "
                     << subscriber.getTopic());

    return subscriber;
}

ros::Subscriber CmdVelSelectorNode::make_autonomous_subscriber() {
    using MessageT = geometry_msgs::Twist;

    static constexpr char TOPIC[] = "autonomous/cmd_vel";
    static constexpr std::uint32_t QUEUE_SIZE = 10;
    static constexpr auto &&CALLBACK_PTR = &CmdVelSelectorNode::add_autonomous;

    auto &&subscriber = node_.subscribe<MessageT>(TOPIC, QUEUE_SIZE,
                                                  CALLBACK_PTR, this);

    ROS_DEBUG_STREAM("subscribed to autonomous cmd_vel on "
                     << subscriber.getTopic());

    return subscriber;
}

ros::Subscriber CmdVelSelectorNode::make_joy_subscriber() {
    using MessageT = sensor_msgs::Joy;

    static const std::string TOPIC = "joy";
    static constexpr std::uint32_t QUEUE_SIZE = 10;
    static constexpr auto &&CALLBACK_PTR = &CmdVelSelectorNode::check_button;

    auto &&subscriber = node_.subscribe<MessageT>(TOPIC, QUEUE_SIZE,
                                                  CALLBACK_PTR, this);

    ROS_DEBUG_STREAM("subscribed to joy on " << subscriber.getTopic());

    return subscriber;
}

ros::Publisher CmdVelSelectorNode::make_cmd_vel_publisher() {
    using MessageT = geometry_msgs::Twist;

    static const std::string TOPIC = "cmd_vel";
    static constexpr std::uint32_t QUEUE_SIZE = 10;
    static constexpr bool LATCHED = false;

    auto &&publisher = node_.advertise<MessageT>(TOPIC, QUEUE_SIZE, LATCHED);

    ROS_DEBUG_STREAM("publishing cmd_vel on " << publisher.getTopic());

    return publisher;
}

ros::Publisher CmdVelSelectorNode::make_source_publisher() {
    using MessageT = std_msgs::String;

    static const std::string TOPIC = "cmd_vel_source";
    static constexpr std::uint32_t QUEUE_SIZE = 10;
    static constexpr bool LATCHED = true;

    auto &&publisher = node_.advertise<MessageT>(TOPIC, QUEUE_SIZE, LATCHED);

    ROS_DEBUG_STREAM("publishing source on " << publisher.getTopic());

    return publisher;
}

ros::Timer CmdVelSelectorNode::make_poll_timer() {
    const ros::Rate rate = get_rate();

    return node_.createTimer(rate, &CmdVelSelectorNode::poll_command, this);
}

ros::Rate CmdVelSelectorNode::get_rate() const {
    static const std::string KEY = "rate";
    static constexpr double DEFAULT = 20.0;

    double fetched;
    local_node_.param(KEY, fetched, DEFAULT);

    if (fetched <= 0) {
        ROS_WARN_STREAM("invalid value of ~rate: " << fetched
                        << ", using default of " << DEFAULT);

        return ros::Rate{ DEFAULT };
    }

    ROS_DEBUG_STREAM("~rate: " << fetched);

    return ros::Rate{ fetched };
}

std::size_t CmdVelSelectorNode::get_joy_button() const {
    static const std::string KEY = "joy_button";
    static constexpr int DEFAULT = 0;

    int fetched;
    local_node_.param(KEY, fetched, DEFAULT);

    if (fetched < 0) {
        ROS_WARN_STREAM("invalid value of ~joy_button: " << fetched
                        << ", using default of " << DEFAULT);

        return static_cast<std::size_t>(DEFAULT);
    }

    ROS_DEBUG_STREAM("~joy_button: " << fetched);

    return static_cast<std::size_t>(fetched);
}

std::int32_t CmdVelSelectorNode::get_pressed_threshold() const {
    static const std::string KEY = "pressed_threshold";
    static constexpr int DEFAULT = 1;

    int fetched;
    local_node_.param(KEY, fetched, DEFAULT);

    if (fetched > std::numeric_limits<std::int32_t>::max()
        || fetched < std::numeric_limits<std::int32_t>::min()) {
        ROS_WARN_STREAM("invalid value of ~pressed_threshold: " << fetched
                        << ", using default of " << DEFAULT);

        return static_cast<std::int32_t>(DEFAULT);
    }

    ROS_DEBUG_STREAM("~pressed_threshold: " << fetched);

    return static_cast<std::int32_t>(fetched);
}

ButtonState CmdVelSelectorNode::get_next_state(const std::int32_t button_data)
const noexcept {
    if (button_data >= pressed_threshold_) {
        return ButtonState::Pressed;
    }

    return ButtonState::Unpressed;
}

OutputMode CmdVelSelectorNode::flip_mode() {
    std_msgs::String source_message;

    switch (selector_.mode()) {
    case OutputMode::Teleoperated: {
        selector_.mode() = OutputMode::Autonomous;
        source_message.data = "autonomous";
        source_publisher_.publish(source_message);

        break;
    } case OutputMode::Autonomous: {
        selector_.mode() = OutputMode::Teleoperated;
        source_message.data = "teleoperated";
        source_publisher_.publish(source_message);

        break;
    }
    }

    return selector_.mode();
}

} // namespace cmd_vel_selector
} // namespace umigv
