#include "NodeC.h"
#include <math.h>

NodeC::NodeC(ros::NodeHandle& nh) : nh_(nh),
                                    as_(nh_, "node_c", boost::bind(&NodeC::execute, this, _1), false),
                                    move_group("arm_torso")
{
    // Start the action server
    as_.start();

    ROS_INFO("node_c started");
    ros::spin();
}

void NodeC::resetArm() {
    geometry_msgs::Pose retracted_pose;
    // Set position values (example values, adjust based on your robot's kinematics)
    retracted_pose.position.x = 0.3;  // Adjust based on your robot's workspace
    retracted_pose.position.y = 0.0;
    retracted_pose.position.z = 0.8;  // Adjust based on your robot's workspace

    // Set orientation values (example values, adjust based on your requirements)
    retracted_pose.orientation.x = 0.0;
    retracted_pose.orientation.y = 0.0;
    retracted_pose.orientation.z = 0.0;
    retracted_pose.orientation.w = 1.0;  // Quaternion representing no rotation

    this->move_group.setPoseTarget(retracted_pose);
    this->move_group.move();


}

// Function that perform the gripper moving
void NodeC::move_gripper(const std::vector<float>& gripper_joint_value, const float& seconds) {
    // Create an head controller action client to move the TIAGo's head
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_client("/gripper_controller/follow_joint_trajectory", true);
    //ROS_INFO("Waiting for action server to start");
    // Wait for the action server to come up
    arm_client.waitForServer(); // Will wait for infinite time
    //ROS_INFO("Action server started, sending goal");

    // Send a goal to the action server
    control_msgs::FollowJointTrajectoryGoal goal;
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("gripper_right_finger_joint");
    goal.trajectory.joint_names.push_back("gripper_left_finger_joint");

    // Only one waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // Trajectory point
    int index = 0;
    // Positions            
    goal.trajectory.points.at(index).positions.resize(2);
    // Velocities
    goal.trajectory.points.at(index).velocities.resize(2);
    for (int j = 0; j < gripper_joint_value.size(); ++j)
    {
        goal.trajectory.points.at(index).positions.at(j) = gripper_joint_value.at(j);
        goal.trajectory.points.at(index).velocities.at(j) = 0.0;
    }
    // To be reached 5 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(seconds);
    arm_client.sendGoal(goal);
    
    // Wait for the action to return
    bool finished_before_timeout = arm_client.waitForResult(ros::Duration(20.0));
    //bool finished_before_timeout = arm_client.waitForResult();
    
    // If the goal finished before the time out the goal status is reported
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = arm_client.getState();
        //ROS_INFO("Action Server finish: %s",state.toString().c_str());
    }
    // Otherwise the user is notified that the goal did not finish in the allotted time.
    else
    {
        //ROS_INFO("Action Server did not finish before the time out");
    }
}

void NodeC::pickAction(const assignment2::PickPlaceGoalConstPtr& goal) {
    // Setting up the move group
    this->move_group.setPoseReferenceFrame("base_footprint");
    this->move_group.setMaxVelocityScalingFactor(1.0);
    this->move_group.setPlannerId("SBLkConfigDefault");
    this->move_group.setPlanningTime(45.0);

    // Set the goal pose
    moveit::core::RobotStatePtr current_state = this->move_group.getCurrentState();
    std::vector<double> initial_joint_group_positions;
    const robot_state::JointModelGroup* joint_model_group = this->move_group.getCurrentState()->getJointModelGroup("arm_torso");
    current_state->copyJointGroupPositions(joint_model_group, initial_joint_group_positions);

    // Round the configuration to 2 decimal places
    for (double& val : initial_joint_group_positions)
    {
        val = ceil(val*100.0)/100.0;
    }

    geometry_msgs::Pose marker_pose = goal->marker_pose;
    geometry_msgs::Pose intermediate_pose;
    geometry_msgs::Pose approach_pose;
    geometry_msgs::Pose grasp_pose;

    // Computing orientatino offset
    tf::Quaternion aQ(
                marker_pose.orientation.x,
                marker_pose.orientation.y,
                marker_pose.orientation.z,
                marker_pose.orientation.w);
    tf::Matrix3x3 aMatrix(aQ);
    double roll, pitch, yaw;
    aMatrix.getRPY(roll, pitch, yaw);

    // Updating just for the green marker
    yaw -= M_PI / 2.0;
    roll = M_PI / 2.0;
    pitch = M_PI / 8.0;

    // Setting up grasp pose orientation
    grasp_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI / 2, 0, 0);

    // Setting up intermediate pose orientation
    intermediate_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI / 2, 0, 0);

    // Setting up approach pose orientation
    approach_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI / 2, 0, 0);

    // Setting up center position to avoid collisions
    marker_pose.position.x += goal->marker_size;

    // Compute the displacement wrt the rotation on angle z -> YAW
    marker_pose.position.x += cos(yaw) * goal->marker_size;
    marker_pose.position.y += sin(yaw) * goal->marker_size/2.0;

    // Updating position for the approach pose and the grasp pose
    approach_pose.position = marker_pose.position;
    grasp_pose.position = marker_pose.position;


    // Compute the displacement wrt the rotation on angle y -> PITCH
    float gripper_size = 0.2;
    float distance = 0.05; // Safe distance from the object 5cm
    approach_pose.position.x -= (cos(pitch) * (gripper_size + distance));
    approach_pose.position.z += (sin(pitch) * (gripper_size + distance));
    //grasp_pose.position.x -= (cos(pitch) * gripper_size);
    //grasp_pose.position.z += (sin(pitch) * gripper_size);
    grasp_pose = approach_pose;
    if(goal->marker_ID == 1) {
        grasp_pose.position.z -= 0.23;
    } else if(goal->marker_ID == 2) {
        grasp_pose.position.z -= 0.178;
    } else if(goal->marker_ID == 3) {
        grasp_pose.position.z -= 0.20;
    }

    // Setting the intermediate pose
    intermediate_pose.position.x = 0.3;
    intermediate_pose.position.y = -0.6;
    intermediate_pose.position.z = 0.98;
    intermediate_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2.0, 0, -M_PI/2.0);

    // Be careful I am avoiding the post code at lines 755-767
    
    // Rounding all position values to 3 decimal places
    approach_pose.position.x = ceil(approach_pose.position.x*1000.0)/1000.0;
    approach_pose.position.y = ceil(approach_pose.position.y*1000.0)/1000.0;
    approach_pose.position.z = ceil(approach_pose.position.z*1000.0)/1000.0;
    grasp_pose.position.x = ceil(grasp_pose.position.x*1000.0)/1000.0;
    grasp_pose.position.y = ceil(grasp_pose.position.y*1000.0)/1000.0;
    grasp_pose.position.z = ceil(grasp_pose.position.z*1000.0)/1000.0;
    intermediate_pose.position.x = ceil(intermediate_pose.position.x*1000.0)/1000.0;
    intermediate_pose.position.y = ceil(intermediate_pose.position.y*1000.0)/1000.0;
    intermediate_pose.position.z = ceil(intermediate_pose.position.z*1000.0)/1000.0;
    
    // Actual moving of the arm, first from the current position to the intermediate pose
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    this->move_group.setStartState(*this->move_group.getCurrentState());
    this->move_group.setPoseTarget(intermediate_pose);
    moveit::core::MoveItErrorCode error_code = this->move_group.plan(my_plan);
    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Plan found in %f seconds", my_plan.planning_time_);
        //  Execute the plan
        error_code = this->move_group.move();
        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Motion completed!");
        }
        else
        {
            ROS_INFO("Motion failed! Error: %i:", error_code.val);
        }
    } else {
        ROS_INFO("Plan failed! Error: %i:", error_code.val);
    }

    // Now for the intermediate pose to the approach pose
    this->move_group.setStartState(*this->move_group.getCurrentState());
    this->move_group.setPoseTarget(approach_pose);
    error_code = this->move_group.plan(my_plan);
    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Plan found in %f seconds", my_plan.planning_time_);
        //  Execute the plan
        error_code = this->move_group.move();
        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Motion completed!");
            ROS_INFO("Reached approach pose!");
        }
        else
        {
            ROS_INFO("Motion failed! Error: %i:", error_code.val);
        }
    } else {
        ROS_INFO("Plan failed! Error: %i:", error_code.val);
    }

    // Define elements to remove
    std::vector<std::string> elemToRemove = {std::to_string(goal->marker_ID), "table"};
    std::vector<std::string> tableList = {"table"};
    std::map<std::string, moveit_msgs::CollisionObject> tableInfos = planning_scene_interface.getObjects(tableList);

    // Remove collision objects
    planning_scene_interface.removeCollisionObjects(elemToRemove);
    ROS_INFO("Collision objects removed");

    // Now for the intermediate pose to the grasp pose
    this->move_group.setStartState(*this->move_group.getCurrentState());
    this->move_group.setPoseTarget(grasp_pose);
    error_code = this->move_group.plan(my_plan);
    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Plan found in %f seconds", my_plan.planning_time_);
        //  Execute the plan
        error_code = this->move_group.move();
        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Motion completed!");
            ROS_INFO("Reached grasp pose!");
        }
        else
        {
            ROS_INFO("Motion failed! Error: %i:", error_code.val);
        }
    } else {
        ROS_INFO("Plan failed! Error: %i:", error_code.val);
    }

    ros::Duration(1.0).sleep();

    auto tableCollisionObjectIterator = tableInfos.find("table");
    if (tableCollisionObjectIterator != tableInfos.end()) {
        moveit_msgs::CollisionObject tableCollisionObject = tableCollisionObjectIterator->second;
        std::vector<moveit_msgs::CollisionObject> tableCollisionObjectList;
        tableCollisionObjectList.push_back(tableCollisionObject);
        // Now, you can re-add the removed collision objects
        this->planning_scene_interface.addCollisionObjects(tableCollisionObjectList);
        ROS_INFO("Table collision object re-added");
    }

    ros::Publisher planning_scene_diff_publisher;
    planning_scene_diff_publisher = this->nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    // Attaching the object with gazebo_ros_link_attacher:w
    ros::ServiceClient attach_client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    gazebo_ros_link_attacher::Attach attach_srv;
    // Set the names of the link to be attached and the object to be attached
    if(goal->marker_ID == 1) // If ID == 1 then attach the blue hexagon
    {
        attach_srv.request.model_name_1 = "Hexagon";
        attach_srv.request.link_name_1 = "Hexagon_link";
    }
    else if(goal->marker_ID == 2) // If ID == 2 then attach the green triangle
    {
        attach_srv.request.model_name_1 = "Triangle";
        attach_srv.request.link_name_1 = "Triangle_link";
    }
    else if(goal->marker_ID == 3) // If ID == 3 then attach the red cube
    {
        attach_srv.request.model_name_1 = "cube";
        attach_srv.request.link_name_1 = "cube_link";
    }
    attach_srv.request.model_name_2 = "tiago";
    attach_srv.request.link_name_2 = "arm_7_link";
    attach_client.call(attach_srv);

    // Close gripper
    std::vector<float> closing_gripper_joint_value = {0.0, 0.0};
    move_gripper(closing_gripper_joint_value, 1.0);


    // Now back to the intermediate pose
    this->move_group.setStartState(*this->move_group.getCurrentState());
    this->move_group.setPoseTarget(intermediate_pose);
    error_code = this->move_group.plan(my_plan);
    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Plan found in %f seconds", my_plan.planning_time_);
        //  Execute the plan
        error_code = this->move_group.move();
        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Motion completed!");
        }
        else
        {
            ROS_INFO("Motion failed! Error: %i:", error_code.val);
        }
    } else {
        ROS_INFO("Plan failed! Error: %i:", error_code.val);
    }


    // And now close the arm back to the body
    this->move_group.setStartState(*this->move_group.getCurrentState());
    this->move_group.setJointValueTarget(initial_joint_group_positions);
    error_code = this->move_group.plan(my_plan);
    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Plan found in %f seconds", my_plan.planning_time_);
        //  Execute the plan
        error_code = this->move_group.move();
        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Motion completed!");
        }
        else
        {
            ROS_INFO("Motion failed! Error: %i:", error_code.val);
        }
    } else {
        ROS_INFO("Plan failed! Error: %i:", error_code.val);
    }

    this->as_.setSucceeded();
}

void NodeC::placeAction(const assignment2::PickPlaceGoalConstPtr& goal) {
    // Setting up the move group
    this->move_group.setPoseReferenceFrame("base_footprint");
    this->move_group.setMaxVelocityScalingFactor(1.0);
    this->move_group.setPlannerId("SBLkConfigDefault");
    this->move_group.setPlanningTime(45.0);

    // Set the goal pose
    moveit::core::RobotStatePtr current_state = this->move_group.getCurrentState();
    std::vector<double> initial_joint_group_positions;
    const robot_state::JointModelGroup* joint_model_group = this->move_group.getCurrentState()->getJointModelGroup("arm_torso");
    current_state->copyJointGroupPositions(joint_model_group, initial_joint_group_positions);

    // Round the configuration to 2 decimal places
    for (double& val : initial_joint_group_positions)
    {
        val = ceil(val*100.0)/100.0;
    }

    geometry_msgs::Pose marker_pose = goal->marker_pose;
    geometry_msgs::Pose intermediate_pose;
    geometry_msgs::Pose approach_pose;
    geometry_msgs::Pose grasp_pose;

    // Computing orientation offset
    tf::Quaternion aQ(
                marker_pose.orientation.x,
                marker_pose.orientation.y,
                marker_pose.orientation.z,
                marker_pose.orientation.w);
    tf::Matrix3x3 aMatrix(aQ);
    double roll, pitch, yaw;
    aMatrix.getRPY(roll, pitch, yaw);

    // Updating just for the green marker
    yaw -= M_PI / 2.0;
    roll = M_PI / 2.0;
    pitch = M_PI / 8.0;
    
    marker_pose.position = goal->marker_pose.position;

    // Setting up grasp pose orientation
    grasp_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI / 2, 0, 0);

    // Setting up intermediate pose orientation
    intermediate_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI / 2, 0, 0);

    // Setting up approach pose orientation
    approach_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI / 2, 0, 0);

    ROS_INFO("Approach pose: %f, %f, %f", marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);

    // Updating position for the approach pose and the grasp pose
    approach_pose.position = marker_pose.position;
    grasp_pose.position = marker_pose.position;


    // Compute the displacement wrt the rotation on angle y -> PITCH
    float gripper_size = 0.2;
    float distance = 0.05; // Safe distance from the object 5cm
    approach_pose.position.x -= (cos(pitch) * (gripper_size + distance));
    approach_pose.position.z += (sin(pitch) * (gripper_size + distance));
    //grasp_pose.position.x -= (cos(pitch) * gripper_size);
    //grasp_pose.position.z += (sin(pitch) * gripper_size);
    grasp_pose = approach_pose;

    grasp_pose.position.z -= 0.10;

    // Setting the intermediate pose
    intermediate_pose.position.x = 0.3;
    intermediate_pose.position.y = -0.6;
    intermediate_pose.position.z = 0.98;
    intermediate_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2.0, 0, -M_PI/2.0);

    // Be careful I am avoiding the post code at lines 755-767
    
    // Rounding all position values to 3 decimal places
    approach_pose.position.x = ceil(approach_pose.position.x*1000.0)/1000.0;
    approach_pose.position.y = ceil(approach_pose.position.y*1000.0)/1000.0;
    approach_pose.position.z = ceil(approach_pose.position.z*1000.0)/1000.0;
    grasp_pose.position.x = ceil(grasp_pose.position.x*1000.0)/1000.0;
    grasp_pose.position.y = ceil(grasp_pose.position.y*1000.0)/1000.0;
    grasp_pose.position.z = ceil(grasp_pose.position.z*1000.0)/1000.0;
    intermediate_pose.position.x = ceil(intermediate_pose.position.x*1000.0)/1000.0;
    intermediate_pose.position.y = ceil(intermediate_pose.position.y*1000.0)/1000.0;
    intermediate_pose.position.z = ceil(intermediate_pose.position.z*1000.0)/1000.0;
    
    // Actual moving of the arm, first from the current position to the intermediate pose
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    this->move_group.setStartState(*this->move_group.getCurrentState());
    this->move_group.setPoseTarget(intermediate_pose);
    moveit::core::MoveItErrorCode error_code = this->move_group.plan(my_plan);
    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Plan found in %f seconds", my_plan.planning_time_);
        //  Execute the plan
        error_code = this->move_group.move();
        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Motion completed!");
        }
        else
        {
            ROS_INFO("Motion failed! Error: %i:", error_code.val);
        }
    } else {
        ROS_INFO("Plan failed! Error: %i:", error_code.val);
    }

    ros::Duration(1.0).sleep();

    // Define elements to remove
    std::string s = "cylinder_table_" + std::to_string(goal->marker_ID);
    std::vector<std::string> elemToRemove = {s};
    std::vector<std::string> tableList = {s};
    std::map<std::string, moveit_msgs::CollisionObject> tableInfos = planning_scene_interface.getObjects(tableList);

    // Remove collision objects
    planning_scene_interface.removeCollisionObjects(elemToRemove);
    ROS_INFO("Collision objects removed");

    // Now for the intermediate pose to the grasp pose
    this->move_group.setStartState(*this->move_group.getCurrentState());
    this->move_group.setPoseTarget(grasp_pose);
    error_code = this->move_group.plan(my_plan);
    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Plan found in %f seconds", my_plan.planning_time_);
        //  Execute the plan
        error_code = this->move_group.move();
        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Motion completed!");
            ROS_INFO("Reached grasp pose!");
        }
        else
        {
            ROS_INFO("Motion to grasp position failed! Error: %i:", error_code.val);
        }
    } else {
        ROS_INFO("Plan for grasp position failed! Error: %i:", error_code.val);
    }

    ros::Duration(1.0).sleep();

    auto tableCollisionObjectIterator = tableInfos.find(s);
    if (tableCollisionObjectIterator != tableInfos.end()) {
        moveit_msgs::CollisionObject tableCollisionObject = tableCollisionObjectIterator->second;
        std::vector<moveit_msgs::CollisionObject> tableCollisionObjectList;
        tableCollisionObjectList.push_back(tableCollisionObject);
        // Now, you can re-add the removed collision objects
        this->planning_scene_interface.addCollisionObjects(tableCollisionObjectList);
        ROS_INFO("Cylindrical table collision object re-added");
    }

    ros::Publisher planning_scene_diff_publisher;
    planning_scene_diff_publisher = this->nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    ros::ServiceClient detach_client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    gazebo_ros_link_attacher::Attach detach_srv;
    // Set the names of the link to be attached and the object to be attached
    if(goal->marker_ID == 1) // If ID == 1 then detach the blue hexagon
    {
        detach_srv.request.model_name_1 = "Hexagon";
        detach_srv.request.link_name_1 = "Hexagon_link";
    }
    else if(goal->marker_ID == 2) // If ID == 2 then detach the green triangle
    {
        detach_srv.request.model_name_1 = "Triangle";
        detach_srv.request.link_name_1 = "Triangle_link";
    }
    else if(goal->marker_ID  == 3) // If ID == 3 then detach the red cube
    {
        detach_srv.request.model_name_1 = "cube";
        detach_srv.request.link_name_1 = "cube_link";
    }
    detach_srv.request.model_name_2 = "tiago";
    detach_srv.request.link_name_2 = "arm_7_link";
    // Call the link_attacher service
    detach_client.call(detach_srv);

    // Open gripper
    std::vector<float> closing_gripper_joint_value = {0.04, 0.04};
    move_gripper(closing_gripper_joint_value, 1.0);
    ros::Duration(2.0).sleep();

    // Now for the intermediate pose to the approach pose
    this->move_group.setStartState(*this->move_group.getCurrentState());
    this->move_group.setPoseTarget(approach_pose);
    error_code = this->move_group.plan(my_plan);
    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Plan found in %f seconds", my_plan.planning_time_);
        //  Execute the plan
        error_code = this->move_group.move();
        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Motion completed!");
            ROS_INFO("Reached approach pose!");
        }
        else
        {
            ROS_INFO("Motion to approach position failed! Error: %i:", error_code.val);
        }
    } else {
        ROS_INFO("Plan for approach position failed! Error: %i:", error_code.val);
    }

    // Now back to the intermediate pose
    this->move_group.setStartState(*this->move_group.getCurrentState());
    this->move_group.setPoseTarget(intermediate_pose);
    error_code = this->move_group.plan(my_plan);
    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Plan found in %f seconds", my_plan.planning_time_);
        //  Execute the plan
        error_code = this->move_group.move();
        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Motion completed!");
        }
        else
        {
            ROS_INFO("Motion to intermediate position failed! Error: %i:", error_code.val);
        }
    } else {
        ROS_INFO("Plan failed for intermediate position! Error: %i:", error_code.val);
    }


    // And now close the arm back to the body
    this->move_group.setStartState(*this->move_group.getCurrentState());
    this->move_group.setJointValueTarget(initial_joint_group_positions);
    error_code = this->move_group.plan(my_plan);
    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Plan found in %f seconds", my_plan.planning_time_);
        //  Execute the plan
        error_code = this->move_group.move();
        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO("Motion completed!");
        }
        else
        {
            ROS_INFO("Motion to initial position failed! Error: %i:", error_code.val);
        }
    } else {
        ROS_INFO("Plan failed for initial position! Error: %i:", error_code.val);
    }

    this->as_.setSucceeded();

}

void NodeC::execute(const assignment2::PickPlaceGoalConstPtr& goal) {
    ROS_INFO("NodeC: Executing goal...");
    ROS_INFO("CHECK THIS: %f, %f, %f", goal->marker_pose.position.x,goal->marker_pose.position.y,goal->marker_pose.position.z);
    if(goal->pickOrPlace) {
        pickAction(goal);
    } else {
        placeAction(goal);
    }
    // Sleep for 4 seconds
    ros::Duration(4.0).sleep();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "node_c");
    ros::NodeHandle nh;

    NodeC node_c(nh);

    return 0;
}