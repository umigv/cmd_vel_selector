#ifndef UMIGV_CMD_VEL_SELECTOR_VELOCITY_SELECTOR_H
#define UMIGV_CMD_VEL_SELECTOR_VELOCITY_SELECTOR_H

#include <geometry_msgs/Twist.h>

#include <boost/optional.hpp>

namespace umigv {
namespace cmd_vel_selector {

enum class OutputMode {
    Teleoperated,
    Autonomous
};

class VelocitySelector {
public:
    void accept_teleop(const geometry_msgs::Twist &cmd_vel);

    void accept_autonomous(const geometry_msgs::Twist &cmd_vel);

    boost::optional<geometry_msgs::Twist> latest() noexcept;

    OutputMode& mode() noexcept;

    OutputMode mode() const noexcept;

private:
    boost::optional<geometry_msgs::Twist> teleop_;
    boost::optional<geometry_msgs::Twist> autonomous_;
    OutputMode mode_ = OutputMode::Teleoperated;
};


} // namespace cmd_vel_selector
} // namespace umigv

#endif
