#ifndef NODEB_H
#define NODEB_H

#include "ros/ros.h"
#include "assignment2/DetectionAction.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class NodeB
{
public:
    NodeB(ros::NodeHandle &nh);

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<assignment2::DetectionAction> as_;

    bool move_head(const float& head_1_joint, const float& head_2_joint);
    geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose, const tf::StampedTransform& transform);
    void executeCallback(const assignment2::DetectionGoalConstPtr &goal);
};

#endif // NODEB_H
