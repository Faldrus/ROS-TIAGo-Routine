#include "NodeB.h"

NodeB::NodeB(ros::NodeHandle &nh) : nh_(nh), as_(nh, "node_b", boost::bind(&NodeB::executeCallback, this, _1), false)
{
    // Start the action server
    as_.start();

    ROS_INFO("NodeB server is ready to receive calls.");
    ros::spin(); // Keep the node alive
}

// Function that transform the poses obtained by "updateArrayTags", from camera frame to base_link frame before sending them to node_a
geometry_msgs::Pose NodeB::transformPose(const geometry_msgs::Pose& pose, const tf::StampedTransform& transform)
{
    tf::Vector3 v(pose.position.x, pose.position.y, pose.position.z);
    v = transform * v;
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    q = transform * q;

    geometry_msgs::Pose tmp_pose;
    tmp_pose.position.x = v.x();
    tmp_pose.position.y = v.y();
    tmp_pose.position.z = v.z();
    tmp_pose.orientation.x = q.x();
    tmp_pose.orientation.y = q.y();
    tmp_pose.orientation.z = q.z();
    tmp_pose.orientation.w = q.w();
    return tmp_pose;
}

bool NodeB::move_head(const float& head_1_joint, const float& head_2_joint)
{
    // Create an head controller action client to move the TIAGo's head
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_client("/head_controller/follow_joint_trajectory", true);
    // ROS_INFO("Waiting for action server to start");
    // Wait for the action server to come up
    head_client.waitForServer(); // Will wait for infinite time
    // ROS_INFO("Action server started, sending goal");

    // Send a goal to the action server
    control_msgs::FollowJointTrajectoryGoal headGoal;
    // The joint names, which apply to all waypoints
    headGoal.trajectory.joint_names.push_back("head_1_joint");
    headGoal.trajectory.joint_names.push_back("head_2_joint");

    headGoal.trajectory.points.resize(1);
    // First trajectory point
    int index = 0;
    // Positions      
    headGoal.trajectory.points.at(index).positions.resize(2);
    headGoal.trajectory.points.at(index).positions.at(0) = head_1_joint;
    headGoal.trajectory.points.at(index).positions.at(1) = head_2_joint;
    // Velocities
    headGoal.trajectory.points.at(index).velocities.resize(2);
    for (int j = 0; j < 2; ++j)
    {
        headGoal.trajectory.points.at(index).velocities.at(j) = 0.0;
    }
    // To be reached 6 second after starting along the trajectory
    headGoal.trajectory.points.at(index).time_from_start = ros::Duration(3.0);

    head_client.sendGoal(headGoal);
    
    // Wait for the action to return
    bool finished_before_timeout = head_client.waitForResult(ros::Duration(30.0));
    
    // If the goal finished before the time out the goal status is reported
    if (finished_before_timeout)
    {
        // actionlib::SimpleClientGoalState state = head_client.getState();
        // headGoal.trajectory.points.at(index).positions.at(0) = 0.0;
        // headGoal.trajectory.points.at(index).positions.at(1) = 0.0;
        // head_client.sendGoal(headGoal);
        return true;
    }
    // Otherwise the user is notified that the goal did not finish in the allotted time.
    else
    {
        ROS_INFO("Action Server did not finish before the time out");
        return false;
    }
}

void NodeB::executeCallback(const assignment2::DetectionGoalConstPtr &goal)
{

    move_head(goal->joint1, goal->joint2);

    // Process the action goal and fill in the result
    ROS_INFO("Received detection goal. Scanning obstacles...");

    assignment2::DetectionResult result_;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    apriltag_ros::AprilTagDetectionArrayConstPtr tag_array_msg = NULL;
    while(tag_array_msg==NULL) {
        tag_array_msg = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", nh_, ros::Duration(5.0));
        // ROS_INFO("Waiting...");
    }

    std::string target_frame = "base_footprint";
    std::string source_frame = tag_array_msg->header.frame_id.c_str(); // "xtion_rgb_optical_frame"
    try {
        listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
    }
    catch(tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
        // Set the action state to aborted
        as_.setAborted(result_);
        return;
    }
    
    for(int i=0; i < tag_array_msg->detections.size(); i++)
    {
        // Check if we found the goal tag at the current position in the array of detections
        if (tag_array_msg->detections.at(i).id.at(0) == goal->marker_ID) {
            // Save the marker ID, size, pose
            result_.marker_ID = tag_array_msg->detections.at(i).id.at(0);
            result_.marker_size = tag_array_msg->detections.at(i).size.at(0);
            result_.marker_pose = transformPose(tag_array_msg->detections.at(i).pose.pose.pose, transform);
        } else {
            // Save the obstacle ID, size, pose
            result_.obs_ID.push_back(tag_array_msg->detections.at(i).id.at(0));
            result_.obs_size.push_back(tag_array_msg->detections.at(i).size.at(0));
            geometry_msgs::Pose tmp = transformPose(tag_array_msg->detections.at(i).pose.pose.pose, transform);
            result_.obs_pose.push_back(tmp);
        }
    }
    ROS_INFO("The robot found %ld obstacles", result_.obs_ID.size());

    this->move_head(0.0, -0.8);

    // Set the action result
    as_.setSucceeded(result_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;

    NodeB node_b(nh);

    return 0;
}
