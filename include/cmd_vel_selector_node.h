#ifndef UMIGV_CMD_VEL_SELECTOR_CMD_VEL_SELECTOR_NODE_H
#define UMIGV_CMD_VEL_SELECTOR_CMD_VEL_SELECTOR_NODE_H

#include "velocity_selector.h"

#include <type_traits>
#include <utility>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

namespace umigv {
namespace cmd_vel_selector {

enum class ButtonState {
    Pressed,
    Unpressed
};

class CmdVelSelectorNode {
public:
    template <
        typename ...Args,
        std::enable_if_t<
            std::is_constructible<ros::NodeHandle, Args...>::value, int
        > = 0
    >
    CmdVelSelectorNode(Args &&...args)
    noexcept(std::is_nothrow_constructible<ros::NodeHandle, Args...>::value)
    : node_(std::forward<Args>(args)...),
      local_node_{ "~" },
      teleop_subscriber_{ make_teleop_subscriber() },
      autonomous_subscriber_{ make_autonomous_subscriber() },
      joy_subscriber_{ make_joy_subscriber() },
      cmd_vel_publisher_{ make_cmd_vel_publisher() },
      source_publisher_{ make_source_publisher() },
      poll_timer_{ make_poll_timer() },
      joy_button_{ get_joy_button() },
      pressed_threshold_{ get_pressed_threshold() } { }

    void add_teleop(const geometry_msgs::Twist::ConstPtr &cmd_vel_ptr);

    void add_autonomous(const geometry_msgs::Twist::ConstPtr &cmd_vel_ptr);

    void check_button(const sensor_msgs::Joy::ConstPtr &joy_ptr);

    void poll_command(const ros::TimerEvent &event);

private:
    ros::Subscriber make_teleop_subscriber();

    ros::Subscriber make_autonomous_subscriber();

    ros::Subscriber make_joy_subscriber();

    ros::Publisher make_cmd_vel_publisher();

    ros::Publisher make_source_publisher();

    ros::Timer make_poll_timer();

    ros::Rate get_rate() const;

    std::size_t get_joy_button() const;

    std::int32_t get_pressed_threshold() const;

    ButtonState get_next_state(std::int32_t button_data) const noexcept;

    OutputMode flip_mode();

    ros::NodeHandle node_;
    ros::NodeHandle local_node_;
    ros::Subscriber teleop_subscriber_;
    ros::Subscriber autonomous_subscriber_;
    ros::Subscriber joy_subscriber_;
    ros::Publisher cmd_vel_publisher_;
    ros::Publisher source_publisher_;
    ros::Timer poll_timer_;
    VelocitySelector selector_;
    std::size_t joy_button_ = 0;
    ButtonState button_state_ = ButtonState::Unpressed;
    std::int32_t pressed_threshold_ = 1;
};

} // namespace cmd_vel_selector
} // namespace umigv

#endif
