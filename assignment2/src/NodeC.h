#ifndef NODEC_H
#define NODEC_H

#include "ros/ros.h"
#include "tiago_iaslab_simulation/Objs.h"
#include "assignment2/PickPlaceAction.h"
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class NodeC {
public:
    NodeC(ros::NodeHandle& nh);
    void execute(const assignment2::PickPlaceGoalConstPtr& goal);

private:
    void pickAction(const assignment2::PickPlaceGoalConstPtr& goal);
    void placeAction(const assignment2::PickPlaceGoalConstPtr& goal);
    void move_gripper(const std::vector<float>& gripper_joint_value, const float& seconds);
    void resetArm();
    void pickPlaceActive();
    void pickPlaceEnded();
    void pickPlaceFeedback(const assignment2::PickPlaceFeedbackConstPtr& feedback);

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<assignment2::PickPlaceAction> as_;
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
};

#endif // NODEC_H
