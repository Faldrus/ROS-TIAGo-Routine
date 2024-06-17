#ifndef USER_NODE_H
#define USER_NODE_H

#include <ros/ros.h>
#include <navigation/CustomAction.h>
#include <actionlib/client/simple_action_client.h>

class UserNode {
public:
    /**
     * @brief Construct a new User Node object
     * 
     * @param argv The params passed from the actual main, see .launch file in navigation for setup
     */
    UserNode(char** argv);

    /**
     * @brief Destroy the User Node object
     */
    ~UserNode();

    /**
     * @brief Prints the current status and position of the robot once received
     * 
     * @param feedback Feedback from the ProxyNode server
     */
    void feedbackCallBack(const navigation::CustomFeedbackConstPtr& feedback);

    /**
     * @brief Prints the end of the goal from the client side
     */
    void goalEnded();

    /**
     * @brief Placeholder function for active goal
     */
    void activeGoal();

private:
    /**
     * @brief Main method which actually controls the logic of the class
     * 
     * @param x_goal Where to send the robot, x axis
     * @param y_goal Where to send the robot, y axis
     * @param orientation Which rotation must have the robot once arrived in x,y
     */
    void sendGoalAndWaitForResult(double x_goal, double y_goal, double orientation);

    char** argv_; // The arguments passed from the main
    actionlib::SimpleActionClient<navigation::CustomAction> ac_; // A simple action client to communicate with ProxyNode
};

#endif  // USER_NODE_H
