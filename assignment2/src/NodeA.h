#ifndef NODEA_H
#define NODEA_H

#include "ros/ros.h"
#include "tiago_iaslab_simulation/Objs.h"
#include <assignment2/DetectionAction.h>
#include <navigation/CustomAction.h>
#include <assignment2/PickPlaceAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <map>

class NodeA {
public:
    NodeA(ros::NodeHandle& nh);
    geometry_msgs::Pose fixPose(const geometry_msgs::Pose& pose);
    geometry_msgs::Pose fixPoseInverse(const geometry_msgs::Pose& pose);
    void setCollisions();
    void detectionGoalEnded();
    void detectionActiveGoal();
    void detectionFeedbackCallBack(const assignment2::DetectionFeedbackConstPtr& feedback);
    void sendHumanNodeRequest();
    void navigateToTarget(std::vector<int> ids);

private:
    void pickPlaceActive();
    void pickPlaceEnded();
    void pickPlaceFeedback(const assignment2::PickPlaceFeedbackConstPtr& feedback);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void detectCylinderTables(const navigation::CustomResultConstPtr result);

    ros::NodeHandle nh_;
    ros::ServiceClient human_service_client_;
    actionlib::SimpleActionClient<navigation::CustomAction> ac_;
    actionlib::SimpleActionClient<assignment2::DetectionAction> bc_;
    actionlib::SimpleActionClient<assignment2::PickPlaceAction> cc_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group_arm;
    image_transport::Subscriber image_sub_;
    cv_bridge::CvImagePtr cv_ptr;
    std::map<int, geometry_msgs::Pose> cylinders_map; // 1: blue, 2: green, 3: red

    geometry_msgs::Pose current_marker_pose;
    int current_marker_ID;
    float current_marker_size;
    bool detected_cylinders;
};

#endif // NODEA_H
