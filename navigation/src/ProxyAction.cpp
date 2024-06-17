#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <navigation/CustomAction.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "ProxyAction.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <navigation/position.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

/**
 * @brief Construct a new Proxy Action:: Proxy Action object
 * The starting values (x,y,yaw) = (0,0,0) are by assumption on the simulation setup
 * Furthermore, the subscription to the move_base/status and scan are here instatiated
 */
ProxyAction::ProxyAction() : as_(nh_, "proxy_action", boost::bind(&ProxyAction::execute, this, _1), false),
                ac_("move_base", true) {
    as_.start();
    motionControlLawActive = false;
    status.x = 0.0;
    status.y = 0.0;
    status.orientation = 0.0;
    status.status_code = -1;
    status_sub_ = nh_.subscribe("move_base/status", 1, &ProxyAction::statusCallBack, this);
    laser_sub_ = nh_.subscribe("scan", 1, &ProxyAction::laserCallBack, this);
    costmap_sub_ = nh_.subscribe("/move_base/local_costmap/costmap", 1, &ProxyAction::costmapCallBack, this);
    tab_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/tab_vel", 1);
}

/**
 * @brief Updates the status.status_code member variable based on /move_base/status
 * 
 * @param status_msg The incoming status from the robot
 * 
 * Have a look at http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalStatus.html
 * for more information on possible inputs
 */
void ProxyAction::statusCallBack(const actionlib_msgs::GoalStatusArrayConstPtr& status_msg) {
    // Processing the robot status and updating the status.status_code member
    // variable accordingly
    if (!status_msg->status_list.empty()) {
        const actionlib_msgs::GoalStatus& latest_status = status_msg->status_list.back();
        switch (latest_status.status) {
            case actionlib_msgs::GoalStatus::SUCCEEDED:
                this->status.status_code = 0;
                break;
            case actionlib_msgs::GoalStatus::ACTIVE:
                this->status.status_code = 1;
                break;
            case actionlib_msgs::GoalStatus::PENDING:
                this->status.status_code = 4;
                break;
            case actionlib_msgs::GoalStatus::REJECTED:
                this->status.status_code = -2;
                break;
            case actionlib_msgs::GoalStatus::PREEMPTED:
                this->status.status_code = 2;
                break;
            case actionlib_msgs::GoalStatus::ABORTED:
                this->status.status_code = 3;
                break;
            case actionlib_msgs::GoalStatus::PREEMPTING:
                this->status.status_code = 5;
                break;
            case actionlib_msgs::GoalStatus::RECALLING:
                this->status.status_code = 6;
                break;
            case actionlib_msgs::GoalStatus::RECALLED:
                this->status.status_code = 7;
                break;
            case actionlib_msgs::GoalStatus::LOST:
                this->status.status_code = 8;
                break;
            default:
                // Handle unknown status
                this->status.status_code = -1;
                break;
        }
    }
}

/**
 * @brief Publishes a goal to the UserNode client
 * 
 * @param centers The centers of the obstales in (x,y) pairs format
 *
 * Simple helper function to improve code readibility
 */
void ProxyAction::publishGoal(const std::vector<std::pair<float, float>>& centers) {
    // Publish the goal with the obstacles
    navigation::CustomResult result_msg;

    // Add the obstacles' centers to the goal message
    for (const auto& center : centers) {
        navigation::position obstacles;
        obstacles.x = center.first;
        obstacles.y = center.second;
        result_msg.obstacles.push_back(obstacles);
    }
    result_msg.result = true;

    // Publish the goal with the obstacles
    as_.setSucceeded(result_msg);
}

/**
 * @brief Outputs the start of the goal
 */
void ProxyAction::activeGoal() {
    ROS_INFO("Started goal in Proxy node");
}

/**
 * @brief Outputs the end of the goal
 */
void ProxyAction::goalEnded() {
    ROS_INFO("Goal endend in Proxy node");
}

/**
 * @brief Outputs the yaw given a quaternion
 * 
 * @param quaternion The input in quaternion format
 * @return float The yaw
 */
float ProxyAction::get2DAngleFromQuaternion(const geometry_msgs::Quaternion& quaternion) {
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);

    // Convert quaternion to yaw angle (z-axis rotation)
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    // Ensure the angle is within the range [0, 2*pi)
    yaw = fmod(yaw + 2 * M_PI, 2 * M_PI);

    return static_cast<float>(yaw);
}

/**
 * @brief Updates the robot position and angle given the move_base informations and propagates info
 * 
 * @param feedback The feedback from the move_base server
 * 
 * Here the member variables regarding position and orientation are updated.
 * Also the new data acquired is propagated to the UserNode client through Proxy own feedback
 */
void ProxyAction::feedbackCallBack(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    if (this->status.status_code == -1) {
        // We have an undefined status, do not send feedback
        return;
    }
    // Setting up the feedback position
    navigation::position robot_pos;
    robot_pos.x = feedback->base_position.pose.position.x;
    robot_pos.y = feedback->base_position.pose.position.y;
    float angle = this->get2DAngleFromQuaternion(feedback->base_position.pose.orientation);

    // Updating member variables
    this->status.x = robot_pos.x;
    this->status.y = robot_pos.y;
    this->status.orientation = angle;

    // Setting up the feedback
    navigation::CustomFeedback feedback_msg;
    feedback_msg.status = this->status.status_code;
    feedback_msg.robot_pos = robot_pos;
    feedback_msg.status = status.status_code;

    // Send the feedback message back to the client
    as_.publishFeedback(feedback_msg);

}

/**
* @brief Executes in loop checkCostmapAndRetakeControl() in order to publish the movement instrusction in tab_vel often enough 
*
*/
void ProxyAction::runControlLoop() {

    ros::Rate loop_rate(5);  // Adjust the loop rate as needed

    while (ros::ok()) {
        ros::spinOnce();  // Process callbacks
        // checkCostmapAndRetakeControl();  // Check costmap and retake control if necessary
        loop_rate.sleep();  // Sleep to control loop rate
    }
}

/**
* @brief Check if the value of the costmap is greater than 0 and, if so, start the self motion control law
*
*/
void ProxyAction::checkCostmapAndRetakeControl() {
    if (cost_value > 0) {
        if(!motionControlLawActive) {
            ROS_WARN("Starting motion control law");
        }
        manualControl();
    } else if (cost_value == 0 && motionControlLawActive) {
        ROS_WARN("Costmap value is zero. Motion control law turned off.");
        stopManualControl();  // Stop manual control if active
    }
}

/**
* @brief Costantly updates the current value of the local costmap
* 
* @param feedback Feedback from the move_base/local_costmap/costmap
*/
void ProxyAction::costmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg) {
    // Access costmap data and print it for the current position of the robot
    int map_width = costmap_msg->info.width;
    int map_height = costmap_msg->info.height;

    // Assuming robot position is given in absolute coordinates
    double robot_x_world = status.x;
    double robot_y_world = status.y;

    // Compute robot position in costmap pixel coordinates
    int robot_x_pixel = static_cast<int>((robot_x_world - costmap_msg->info.origin.position.x) / costmap_msg->info.resolution);
    int robot_y_pixel = static_cast<int>((robot_y_world - costmap_msg->info.origin.position.y) / costmap_msg->info.resolution);

    // Ensure the robot position is within the costmap bounds
    if (robot_x_pixel >= 0 && robot_x_pixel < map_width && robot_y_pixel >= 0 && robot_y_pixel < map_height) {
        int index = robot_y_pixel * map_width + robot_x_pixel;
        cost_value = costmap_msg->data[index];
    } else {
        ROS_ERROR("Robot is outside the bounds of the local costmap.");
    }
}

/**
* @brief Implements the self motion control law. It finds the farthest point from the measurements of the laserscan and point the robot towards its direction.
*
*/
void ProxyAction::manualControl() {

    motionControlLawActive = true;
    geometry_msgs::Twist tab_vel;
    
    // Check if laser data is available
    if (!laser_data.ranges.empty()) {
        // Check for the maximum distance
        float maxDistance = laser_data.ranges[0];
        int maxIndex = 0;
        for(int i = 0; i < laser_data.ranges.size(); i++) {
            if(laser_data.ranges[i] > maxDistance) {
                maxDistance = laser_data.ranges[i];
                maxIndex = i;
            }
        }

        // Compute the target point
        float theta = maxIndex * laser_data.angle_increment + laser_data.angle_min;
        float target_x = maxDistance * std::cos(theta);
        float target_y = maxDistance * std::sin(theta);

        // Calculate linear and angular velocities
        float distance_to_target = std::sqrt(std::pow(target_x, 2) + std::pow(target_y, 2));
        float angle_to_target = std::atan2(target_y, target_x);

        // Set linear velocity
        tab_vel.linear.x = 0.5; // You can adjust this value as needed

        // Set angular velocity
        float K_theta = 0.5; // You may need to adjust this value
        tab_vel.angular.z = K_theta * angle_to_target;

        // Publish the tab_vel message to control the robot
        tab_vel_pub_.publish(tab_vel);
    }
}

/**
* @brief Stops the self motion control law
*
*/
void ProxyAction::stopManualControl() {
    motionControlLawActive = false;
    // Stop manual control by publishing zero velocities
    geometry_msgs::Twist stop_tab_vel;
    tab_vel_pub_.publish(stop_tab_vel);

    // After stopping manual control, re-send the original goal
    if (ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE) {

        ROS_INFO("Resending the original goal.");

        // Create a goal
        move_base_msgs::MoveBaseGoal goalToSend;

        // Converting orientation angle in degrees to quaternion
        tf2::Quaternion quat;
        quat.setRPY(0, 0, goal_to_send.orientation * M_PI / 180.0);

        // Set the goal pose
        goalToSend.target_pose.header.frame_id = "map";
        goalToSend.target_pose.header.stamp = ros::Time::now();
        goalToSend.target_pose.pose.position.x = goal_to_send.x_goal;
        goalToSend.target_pose.pose.position.y = goal_to_send.y_goal;
        goalToSend.target_pose.pose.orientation = tf2::toMsg(quat);

        // Send the goal
        ac_.sendGoal(goalToSend,
                     boost::bind(&ProxyAction::goalEnded, this),
                     boost::bind(&ProxyAction::activeGoal, this),
                     boost::bind(&ProxyAction::feedbackCallBack, this, _1));
    }
}

/**
 * @brief This function starts the main logic of the whole class as soon as a goal is received
 * 
 * @param goal The final goal to obtain
 * 
 * It starts correctly the action client towards move_base and propagates the goal received from
 * UserNode to the actual robot. Also, once the result are available the function propagates them
 * through ProxyAction own feedbacka and result network.
 */
void ProxyAction::execute(const navigation::CustomGoalConstPtr &goal) {
    // Store the goal to be re-sent if needed
    goal_to_send = *goal;

    // Wait for the action server to come up
    while (!ac_.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Create a goal
    move_base_msgs::MoveBaseGoal goalToSend;

    // Converting orientation angle in degrees to quaternion
    tf2::Quaternion quat;
    quat.setRPY(0, 0, goal->orientation * M_PI / 180.0);

    // Set the goal pose
    goalToSend.target_pose.header.frame_id = "map";
    goalToSend.target_pose.header.stamp = ros::Time::now();
    goalToSend.target_pose.pose.position.x = goal->x_goal;
    goalToSend.target_pose.pose.position.y = goal->y_goal;
    goalToSend.target_pose.pose.orientation = tf2::toMsg(quat);

    // Feedback callback setup during the goal sending
    ac_.sendGoal(goalToSend,
                boost::bind(&ProxyAction::goalEnded, this),
                boost::bind(&ProxyAction::activeGoal, this),
                boost::bind(&ProxyAction::feedbackCallBack, this, _1));

    // Check if the goal was successful
    if (ac_.waitForResult()) {
        // Settin up the feedback
        navigation::position robot_pos;
        robot_pos.x = this->status.x;
        robot_pos.y = this->status.y;

        navigation::CustomFeedback feedback_msg;
        feedback_msg.status = this->status.status_code;
        feedback_msg.robot_pos = robot_pos;
        feedback_msg.status = status.status_code;


        if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            // Starting to scan
            feedback_msg.status = 2;
            as_.publishFeedback(feedback_msg);

            // Process the scanned data
            std::vector<std::pair<float, float>> centers = this->getCentersObstacles();
            
            // Detection ended
            feedback_msg.status = 3;
            as_.publishFeedback(feedback_msg);
            
            // Publish the goal with the obstacles
            this->publishGoal(centers);

            // Setting idle status
            feedback_msg.status = 4;
            as_.publishFeedback(feedback_msg);
        } else {
            ROS_WARN("The robot failed to reach the goal or timed out...");
            // Send result back to the user
            navigation::CustomResult result_msg;
            result_msg.result = false;
            as_.setSucceeded(result_msg);
        }
    } else {
        ROS_ERROR("Failed to get result from move_base action server.");
        // Send result back to the user
        navigation::CustomResult result_msg;
        result_msg.result = false; 
        as_.setSucceeded(result_msg);
    }

}

/**
 * @brief Updates the member variables of the class with new laserscan data
 * 
 * @param scan_msg The incoming scan message from the laser scanner
 *
 * This function filters the input data from the sensor to avoid reading the 
 * arms of the robot too. Then the value from the laser scanner are saved in the
 * LaserData struct, ready to be used for the obstacles computation.
 */
void ProxyAction::laserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    std::vector<float> updated_ranges;
    // Removing robot's arms
    for (int i = 20; i < scan_msg->ranges.size() - 20; i++) {
        updated_ranges.push_back(scan_msg->ranges[i]);
    }
    laser_data.ranges = updated_ranges;
    // Updating angle_min and angle_max taking into account the first 20 and last 20 values are missing
    laser_data.angle_min = scan_msg->angle_min + 20 * scan_msg->angle_increment;
    laser_data.angle_max = scan_msg->angle_max - 20 * scan_msg->angle_increment;
    laser_data.angle_increment = scan_msg->angle_increment;
    laser_data.range_min = scan_msg->range_min;
    laser_data.range_max = scan_msg->range_max;
}

/**
 * @brief Computes the centers of the obstacles based on the laser data
 * 
 * @return A vector of pairs (x,y) which represent the values of the obstacles.
 *
 * The algorithm follows this specific pattern:
 *  1. Based on relative distance between points, categorize as background whatever is
       further behind and seems disconnected from other relevant parts in term of absolute distance
       measured by the scanner
    2. Ensure the partition background-foreground is actually correct by computing the average distance from the robot
       and taking the further partition as background
    3. Suppress foreground instances with less than 5 points measured
    4. Collect the remaining points in shapes based on their closeness. At this point the data is not saved any more in (rho,theta)
       format but in (x,y) format with respect to the initial system of reference in which the robot was spawned.
    5. Given the obstacles have circular base, see which of the shapes least resembles a half-circle (we only see one half)
       and eliminate shapes based on a threshold
    6. Save points to a .txt file that can be found usually under /home/.ros/points.txt for useful debugging and plotting
    7. Return the computed centers
 */
std::vector<std::pair<float, float>> ProxyAction::getCentersObstacles() {
    // Filtering data based on how far they are from each other
    // Basically if a segment of close points is followed by a dramatic increase in distance
    // we change from background to foreground and viceversa.
    std::vector<float> ranges = this->laser_data.ranges;
    std::vector<int> binary_map(ranges.size(), 0); // 0 is valid, 0 is background
    float threshold = 0.8;
    // Assuming we always start with background
    // NOTE: this bias is taken into account later on
    bool is_background = true; 

    for (int i = 0; i < ranges.size(); i++) {
        if (i >= 1) {
            // We can see previous points
            if (std::abs(ranges[i] - ranges[i-1]) > threshold) {
                // There is a noticeable gap
                if (is_background) {
                    // We were analyzing background
                    binary_map[i] = 1;
                    is_background = false;
                } else {
                    // We were analyzing foreground
                    binary_map[i] = 0;
                    is_background = true;
                }
            } else {
                // No noticeable gap, keep label
                binary_map[i] = binary_map[i-1];
            }
        } else {
            // i=0, first element
            binary_map[i] = 0;
        }
    }

    // We might have started on background but it is a strong assumption
    // Now we check which of the two partitions is actually closer to the robot
    // and set the other as background
    float avg_dist_1 = 0;
    int count_1 = 0;
    float avg_dist_0 = 0;
    int count_0 = 0;

    for (int i = 0; i < ranges.size(); i++) {
        if (binary_map[i] == 0) {
            avg_dist_0 += ranges[i];
            count_0++;
        } else {
            avg_dist_1 += ranges[i];
            count_1++;
        }
    }
    avg_dist_0 /= static_cast<float>(count_0);
    avg_dist_1 /= static_cast<float>(count_1);

    int background_index = 0; // from now on this decides which is the background (no 0 or 1)
    if (avg_dist_1 >= avg_dist_0) {background_index = 1;}


    // Set to background_index short burst of distances
    int start_index = -1; // -1 means we are not currently counting
    int min_length = 5; // at least 5 points in a row to be considered valuable
    for (int i = 0; i < binary_map.size(); i++) {
        // For each point
        if (binary_map[i] != background_index && start_index == -1) {
            // It is not background and we are not counting
            start_index = i;
        } else if (binary_map[i] == background_index && start_index != -1) {
            // It is background and we were counting
            if ((i - start_index) <= min_length) {
                // Less than min_length points in a row
                for (int j = start_index; j < i; j++) {
                    binary_map[j] = background_index; // background segment
                }
            } else {
                // More than min_length points in a row
                start_index = -1;
            }
        }
        // Here it is background and we are not counting
        // or it is not background and we are counting
        // nothing to do
    }


    // Collecting segments into separated batches
    std::vector<std::vector<std::pair<float,float>>> shapes; // vector containing points clusters
    for (int i = 0; i < ranges.size(); i++) {
        if (binary_map[i] != background_index) {
            std::vector<std::pair<float,float>> shape;
            while (i < ranges.size() && binary_map[i] != background_index) {
                // Current angle
                float angle = this->laser_data.angle_min + i * this->laser_data.angle_increment;
                // X, Y coordinates with respect to the robot's current POV
                float x_r = ranges[i] * std::cos(angle);
                float y_r = ranges[i] * std::sin(angle);
                // X, Y with respect to the global robot frame
                float x = x_r * std::cos(this->status.orientation) - y_r * std::sin(this->status.orientation) + this->status.x;
                float y = x_r * std::sin(this->status.orientation) + y_r * std::cos(this->status.orientation) + this->status.y;
                shape.push_back(std::make_pair(x,y));
                i++;
            }
            shapes.push_back(shape);
        }
    }

    // Trying to filter based on radius
    // Basically we approximate the center of each shape as the avg point between the first point
    // of the shape and the last. Then we compute a plausible radius and see how much the shape differs
    // on average per point from such radius. The further we are from 0 the less probably the shape
    // is a circle
    std::vector<std::pair<float,float>> centers; // vector containing points clusters
    float res_threshold = 0.3; // How close to a circle it has to be
    std::vector<std::pair<float,float>> scanners; // useful for debugging purposes
    for (int i = 0; i < shapes.size(); i++) {
        auto start = shapes[i][0];
        auto end = shapes[i][shapes[i].size()-1];

        float radius = 0.21;

        // Middle point between start and end
        std::pair<float,float> middle((start.first+end.first)/2.0, (start.second+end.second)/2.0);

        // Compute the slope m
        float m = (end.second - start.second) / (end.first - start.first);

        // Compute the distance between start and end
        float d = std::sqrt(std::pow(start.first-end.first, 2) + std::pow(start.second-end.second, 2));

        // Compute distance from middle to center
        float h = std::sqrt(std::pow(radius, 2) - std::pow(d/2.0, 2));

        // Compute the center
        float x_center = middle.first + h * (end.second - start.second) / d;
        float y_center = middle.second - h * (end.first - start.first) / d;
        std::pair<float,float> center(x_center, y_center);


        // // Defining radius and center of the hypothetic circle
        // float radius = std::sqrt(std::pow(start.first-end.first, 2) + std::pow(start.second-end.second, 2)) / 2.0;
        // std::pair<float,float> center((start.first+end.first)/2.0, (start.second+end.second)/2.0);
        
        // Computing the average residual of the shape
        float residuals = 0;
        for (auto& point : shapes[i]) {
            float dist = std::sqrt(std::pow(point.first-center.first, 2) + std::pow(point.second-center.second, 2)) / 2.0;
            residuals += dist;
        }

        residuals /= static_cast<float>(shapes[i].size());
        // printf("Center: (%f, %f) - Radius: %f - Residuals: %f\n", center.first, center.second, radius, residuals);

        if (residuals <= res_threshold) {
            // It is a match!
            for (auto& point : shapes[i]) {
                scanners.push_back(point);
            }
            scanners.push_back(center);
            centers.push_back(center);
        }
    }

    // save points to a text file
    std::ofstream outfile("points.txt");
    if (outfile.is_open()) {
       for (const auto& point : scanners) {
           outfile << point.first << " " << point.second << std::endl;
       }
       outfile.close();
       std::cout << "points saved to points.txt" << std::endl;
    } else {
       std::cerr << "error opening points.txt for writing" << std::endl;
    }

    return centers;
}
