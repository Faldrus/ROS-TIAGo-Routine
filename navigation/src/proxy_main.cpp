#include <ros/ros.h>
#include "ProxyAction.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "proxy_node");

    ProxyAction proxy;

    proxy.runControlLoop();

    ROS_INFO("Proxy node is ready to receive goals.");

    ros::spin();

    return 0;
}
