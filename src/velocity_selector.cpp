#include "velocity_selector.h"

namespace umigv {
namespace cmd_vel_selector {

void VelocitySelector::accept_teleop(const geometry_msgs::Twist &cmd_vel) {
    teleop_.emplace(cmd_vel);
}

void VelocitySelector::accept_autonomous(const geometry_msgs::Twist &cmd_vel) {
    autonomous_.emplace(cmd_vel);
}

boost::optional<geometry_msgs::Twist> VelocitySelector::latest() noexcept {
    boost::optional<geometry_msgs::Twist> cmd_vel;

    switch (mode_) {
    case OutputMode::Teleoperated: {
        if (!teleop_) {
            return boost::none;
        }

        cmd_vel = std::move(teleop_.value());
        teleop_ = boost::none;

        break;
    } case OutputMode::Autonomous: {
        if (!autonomous_) {
            return boost::none;
        }

        cmd_vel = std::move(autonomous_.value());
        autonomous_ = boost::none;

        break;
    }
    }

    return cmd_vel;
}

OutputMode& VelocitySelector::mode() noexcept {
    return mode_;
}

OutputMode VelocitySelector::mode() const noexcept {
    return mode_;
}

} // namespace cmd_vel_selector
} // namespace umigv
