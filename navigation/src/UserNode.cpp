#include "UserNode.h"
#include "ros/ros.h"
#include "navigation/CustomAction.h"
#include "navigation/position.h"

/**
 * @brief Construct a new User Node:: User Node object
 * 
 * @param argv The actual argv passed from the main function. Have a look at the launch file
 * 
 * Setup the action client towards proxy_action and once started start the whole process
 * through sengGoalAndWaitForResult
 */
UserNode::UserNode(char** argv) : ac_("proxy_action", true) {
    // Wait for the action server to start
    ROS_INFO("Waiting for action server to start...");
    ac_.waitForServer();

    ROS_INFO("Action server started, sending goal to proxy...");

    // Extract X and Y goals from command-line arguments
    double x_goal = std::stod(argv[1]);
    double y_goal = std::stod(argv[2]);
    double orientation = std::stod(argv[3]);

    // Send the goal to the action server and wait for the result
    sendGoalAndWaitForResult(x_goal, y_goal, orientation);
}

/**
 * @brief Destroy the User Node:: User Node object
 */
UserNode::~UserNode() {
    // Destructor
}

/**
 * @brief Main method which actually controls the logic of the class
 * 
 * @param x_goal Where to send the robot, x axis
 * @param y_goal Where to send the robot, y axis
 * @param orientation Which rotation must have the robot once arrived in x,y
 *
 * Sends a goal to the ProxyAction server with the passed parameters.
 * Furthermore it awaits for the conclusion of the result
 */
void UserNode::sendGoalAndWaitForResult(double x_goal, double y_goal, double orientation) {
    // Create a goal for the robot
    navigation::CustomGoal goal;
    goal.x_goal = x_goal;
    goal.y_goal = y_goal;
    goal.orientation = orientation;

    // Send the goal to the action server
    ac_.sendGoal(goal,
                boost::bind(&UserNode::goalEnded, this),
                boost::bind(&UserNode::activeGoal, this),
                boost::bind(&UserNode::feedbackCallBack, this, _1));

    // Wait for the result
    ac_.waitForResult();
}

/**
 * @brief Prints the ending of the goal and the obstacles perceived
 * 
 * The result retrieved from ProxyNode are actually printed to screen
 */
void UserNode::goalEnded() {
    auto result = ac_.getResult();
    if (result->result) {
        ROS_INFO("The Action finished successfully!");
        std::vector<navigation::position> obstacles = result->obstacles;
        for (auto& obstacle : obstacles) {
            printf("Found obstacle with center (%.3f,%.3f)\n", obstacle.x, obstacle.y);
        }
    } else {
        ROS_WARN("There was an error in the actuation of the goal");
    }
}

/**
 * @brief Placeholder function for active goal
 */
void UserNode::activeGoal() {}

/**
 * @brief Prints the robot status and position based on the feedback
 * 
 * @param feedback Feedback from ProxyAction
 */
void UserNode::feedbackCallBack(const navigation::CustomFeedbackConstPtr& feedback) {
    navigation::position robot_pos = feedback->robot_pos;
    int status = feedback->status;

    if(status == 0) {
        ROS_INFO("The robot has stopped in position (%.4f,%.4f)", robot_pos.x, robot_pos.y);
    } else if(status == 1) {
        ROS_INFO("The robot is moving. Its current position is: (%.4f,%.4f)", robot_pos.x, robot_pos.y);
    } else if(status == 2) {
        ROS_INFO("The robot has started the detection. Its current position is: (%.4f,%.4f)", robot_pos.x, robot_pos.y);
    } else if(status == 3) {
        ROS_INFO("The robot has finished the detection. Its current position is: (%.4f,%.4f)", robot_pos.x, robot_pos.y);
    } else if(status == 4) {
        ROS_INFO("The robot is in idle state. Its current position is: (%.4f,%.4f)", robot_pos.x, robot_pos.y);
    } else if(status == -1) {
        ROS_WARN("The robot is in an unknown state. Its current position is: (%.4f,%.4f)", robot_pos.x, robot_pos.y);
    } else {
        ROS_WARN("The robot is in an error state. Its current position is: (%.4f,%.4f)", robot_pos.x, robot_pos.y);
    }
}