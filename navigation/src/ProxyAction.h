#ifndef PROXY_ACTION_H
#define PROXY_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <navigation/CustomAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>

class ProxyAction {
public:
    /**
     * @brief Simple data structure used to store laserscan data
     */
    struct LaserData {
        std::vector<float> ranges; // distances measured by sensor
        float angle_min; // starting angle
        float angle_max; // end angle
        float angle_increment; // angle increment per distance measured
        float range_min;
        float range_max;
    };

    /**
     * @brief Simple data structure used to keeptrack of some robot states
     */
    struct RobotData {
        float x; // current x position of the robot
        float y; // current y position of the robot
        float orientation; // current orientation (yaw) of the robot in rad
        int status_code; // current status of the robot
        /**
            The status code can have the following values:
            # The status variable represents the status of the tiago robot
            # 0: robot has stopped
            # 1: robot is moving
            # 2: robot started detection
            # 3: detection finished
            # 4: idle
            # -1: Unknown state
            # -2: Error state
        */
    };

    /**
     * @brief Construct a new Proxy Action object
     * Initializes correctly all member variables
     */
    ProxyAction();

    /**
     * @brief Signals to output the activation of the goal
     * 
     */
    void activeGoal();

    /**
     * @brief Signals to output the end of the goal
     * 
     */
    void goalEnded();

    /**
     * @brief Publishes the final goal given the centers of the obstacles
     * 
     * @param obstacle_centers Centers of the obstacles detected in pairs (x,y)
     */
    void publishGoal(const std::vector<std::pair<float, float>>& obstacle_centers);

    /**
     * @brief Given feedback from move_base updates the values of the robot and propagates info
     * 
     * @param feedback Feedback from the move_base server
     */
    void feedbackCallBack(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    /**
     * @brief Executes in loop checkCostmapAndRetakeControl() in order to publish the movement instrusction in tab_vel often enough 
     *
     */
    void runControlLoop();

    /**
     * @brief Check if the value of the costmap is greater than 0 and, if so, start the self motion control law
     *
     */
    void checkCostmapAndRetakeControl();

    /**
     * @brief Costantly updates the current value of the local costmap
     * 
     * @param feedback Feedback from the move_base/local_costmap/costmap
     */
    void costmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg);

    /**
     * @brief Implements the self motion control law. It finds the farthest point from the measurements of the laserscan and point the robot towards its direction.
     *
     */
    void manualControl();

    /**
     * @brief Stops the self motion control law
     *
     */
    void stopManualControl();

    /**
     * @brief Given a specific CustomGoal, starts the whole routine for the proxy node
     * 
     * @param goal A suitable CustomGoal that has to be shipped to the robot
     */
    void execute(const navigation::CustomGoalConstPtr& goal);

    /**
     * @brief Updates the status of the proxy node based on the status of the robot
     * 
     * @param status_msg The status message from /move_base/status
     */
    void statusCallBack(const actionlib_msgs::GoalStatusArrayConstPtr& status_msg);

    /**
     * @brief Updates the laser data of the proxy node based on the actual scans of the laser
     * 
     * @param scan_msg The scan message from /scan
     */
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

private:
    /**
     * @brief Compute obstacles' centers based on the laser data
     * 
     * @return std::vector<std::pair<float,float>> obstacles' centers in (x,y) pairs format
     */
    std::vector<std::pair<float,float>> getCentersObstacles();

    /**
     * @brief Computes the yaw in rads of a given quaternion
     * 
     * @param quaternion The input quaternion
     * @return float The yaw in radiants
     */
    float get2DAngleFromQuaternion(const geometry_msgs::Quaternion& quaternion);

    ros::NodeHandle nh_; // main node handle
    actionlib::SimpleActionServer<navigation::CustomAction> as_; // action server to communicate with UserNode
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_; // action client to communicate with the robot
    ros::Subscriber status_sub_; // subscriber to the robot status 
    ros::Subscriber laser_sub_; // subscriber to the laser scanner 
    ros::Subscriber costmap_sub_; // subscriber to the costmap
    ros::Publisher tab_vel_pub_; // publisher for sending velocity commands 
    RobotData status; // Stores main info about the robot
    LaserData laser_data; // Stores main info about the laser data
    navigation::CustomGoal goal_to_send; // goal to send to the robot
    bool motionControlLawActive; // boolean value to know if the robot is moving with the developed self motion control
    unsigned char cost_value; // current value of the local cost map
};

#endif  // PROXY_ACTION_H
