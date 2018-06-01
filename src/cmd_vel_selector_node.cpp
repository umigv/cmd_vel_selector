#include <ros/ros.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "cmd_vel_selector_node");
    ros::NodeHandle node;

    ros::spin();
}
