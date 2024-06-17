#include "ros/ros.h"
#include "UserNode.h"

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "user_node");

    // Create an instance of UserNode
    UserNode user_node(argv);

    return 0;
}
