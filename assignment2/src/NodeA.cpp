#include "NodeA.h"

NodeA::NodeA(ros::NodeHandle& nh) : nh_(nh), ac_("proxy_action", true), bc_("node_b", true), cc_("node_c", true),
                        group_arm("arm_torso") {
    // Initialize the service client
    this->human_service_client_ = this->nh_.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
    // Wait for the action server to start
    ROS_INFO("Waiting for server_node to start...");
    this->ac_.waitForServer();

    // Subscribe to the Tiago camera image topic
    image_transport::ImageTransport it(nh_);
    image_sub_ = it.subscribe("/xtion/rgb/image_color", 1, &NodeA::imageCallback, this);

    // We have not already detected the cylinders
    this->detected_cylinders = false;

    ROS_INFO("node_a started");
}

void NodeA::sendHumanNodeRequest() {
    tiago_iaslab_simulation::Objs srv;
    // If true human_node send the object's ID
    srv.request.ready = true;
    // Set true only for extra point
    srv.request.all_objs = true;
    // Save the sequence of ID to pick up
    std::vector<int> ids;

    if (this->human_service_client_.call(srv)) {
        ROS_INFO("ID size: %lu", srv.response.ids.size());
        // Print on terminal our response
        for (size_t i = 0; i < srv.response.ids.size(); i++) {
            ids.push_back(srv.response.ids.at(i));
            ROS_INFO("ID %zu: %i", i + 1, srv.response.ids.at(i));
        }
        this->navigateToTarget(ids);
    } else {
        ROS_ERROR("Failed to call human_node service");
    }
}

// Callback function for the Tiago camera image
void NodeA::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Convert the ROS image message to a cv::Mat using cv_bridge
        this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// Function used to fix pose of table and cylindrical place table wrt base_footprint
geometry_msgs::Pose NodeA::fixPose(const geometry_msgs::Pose& pose) {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    try {
        listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
    }
    catch(tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
    }

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

geometry_msgs::Pose NodeA::fixPoseInverse(const geometry_msgs::Pose& pose) {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    try {
        listener.waitForTransform("base_footprint", "map", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("base_footprint", "map", ros::Time(0), transform);
    }
    catch(tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
    }

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

void NodeA::setCollisions() {

    auto results = this->bc_.getResult();
    this->current_marker_pose = results->marker_pose;
    this->current_marker_ID = results->marker_ID;
    this->current_marker_size = results->marker_size;

    // Creating environment
    // Create vector to hold n+1 collision objects (+1 is the table)
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    // 1 for the table and 1 for the target object to pick
    int number_objects = results->obs_ID.size() + 1 + 1;
    collision_objects.resize(number_objects);

    // Define the id of the object, which is used to identify it.
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = this->group_arm.getPlanningFrame();

    // Define the primitive and its dimensions
    // Define the table (box) and add to the world.
    // Enlarge the table x,y position to avoid contact with it
    float enlargment_table = 0.25;
    float enlargment_table_z = 0.01;

    // Define the pose of the table (box)
    // It should be wrt the center of the object
    // Transform given costant and predifined value from map to base_footprint
    geometry_msgs::Point table_point;
    table_point.x = 7.826;
    table_point.y = -2.983;
    table_point.z = 0.775 / 2.0;
    geometry_msgs::Point table_surface_position;
    table_surface_position.x = 0.913;
    table_surface_position.y = 0.913;
    table_surface_position.z = 0.795;
    geometry_msgs::Pose table_pose;
    table_pose.position = table_point;
    table_pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    table_pose.position.z += enlargment_table_z/2.0;

    shape_msgs::SolidPrimitive primitive_table;
    primitive_table.type = primitive_table.BOX;
    primitive_table.dimensions.resize(3);
    primitive_table.dimensions[0] = table_surface_position.x + enlargment_table;
    primitive_table.dimensions[1] = table_surface_position.y + enlargment_table;
    primitive_table.dimensions[2] = table_surface_position.z + enlargment_table_z;

    collision_objects[0].primitives.push_back(primitive_table);
    collision_objects[0].primitive_poses.push_back(table_pose);
    collision_objects[0].operation = collision_objects[0].ADD;

    // Define the target object
    collision_objects[1].header.frame_id = this->group_arm.getPlanningFrame();
    collision_objects[1].id = std::to_string(results->marker_ID);

    // Enlarge the the object to be picked on z to void contact with it
    float enlargment_blue_height = 0.1;
    float enlargment_red_height = 0.04;

    // Define the primitive and its dimensions
    shape_msgs::SolidPrimitive primitive_target;
    // Standard value, update for the specific object
    float target_heigth_z = results->marker_pose.position.z - table_surface_position.z;
    float target_pose_z = table_surface_position.z + (target_heigth_z/2.0); 

    geometry_msgs::Pose marker_pose = this->fixPose(results->marker_pose);    
    // Check if the target ID correspond to a red cube (3)
    if (results->marker_ID == 3)
    {
        target_heigth_z = marker_pose.position.z + enlargment_red_height - table_surface_position.z;
        target_pose_z = table_surface_position.z + (target_heigth_z/2.0);
        primitive_target.type = primitive_target.BOX;
        primitive_target.dimensions.resize(3);
        primitive_target.dimensions[0] = results->marker_size*2.0;
        primitive_target.dimensions[1] = results->marker_size*2.0;
        primitive_target.dimensions[2] = target_heigth_z;
    } else if (results->marker_ID == 1) { // Check if the target ID correspond to a blue hexagon (1)
        primitive_target.type = primitive_target.CYLINDER;
        target_heigth_z = marker_pose.position.z + enlargment_blue_height - table_surface_position.z;
        target_pose_z = table_surface_position.z + (target_heigth_z/2.0);
        primitive_target.dimensions.resize(2);
        primitive_target.dimensions[0] = target_heigth_z; // Height
        primitive_target.dimensions[1] = results->marker_size; // Radius
    } else { // Check if the target ID correspond to a green triangle (2)
        std::vector <std::string> a = {std::to_string(results->marker_ID)};
        this->planning_scene_interface.removeCollisionObjects(a);
        primitive_target.type = primitive_target.CONE;
        target_heigth_z = marker_pose.position.z + results->marker_size - table_surface_position.z;
        target_pose_z = table_surface_position.z;
        primitive_target.dimensions.resize(2);
        primitive_target.dimensions[0] = target_heigth_z; // Height
        primitive_target.dimensions[1] = results->marker_size; // Radius
    }

    // Define the pose of the target object
    geometry_msgs::Pose target_object_pose;
    target_object_pose.position.x = marker_pose.position.x;
    target_object_pose.position.y = marker_pose.position.y;
    target_object_pose.position.z = target_pose_z;
    target_object_pose.orientation = marker_pose.orientation;

    collision_objects[1].primitives.push_back(primitive_target);
    collision_objects[1].primitive_poses.push_back(target_object_pose);
    collision_objects[1].operation = collision_objects[1].ADD;

    // Define the collision objects for the obstacles to avoid
    // 30cm enlargment for heigth and 4cm for radius
    float enlargment_obstacle_height = 0.07;
    float enlargment_obstacle_radius = 0.02;
    for(int i = 2, j=0; i < number_objects; i++, j++)
    {
        collision_objects[i].header.frame_id = this->group_arm.getPlanningFrame();
        collision_objects[i].id = std::to_string(results->obs_ID.at(j));

        float obstacle_heigth_z = results->obs_pose.at(j).position.z - table_surface_position.z;
        float obstacle_pose_z = table_surface_position.z + (obstacle_heigth_z/2.0);

        // Define the primitive and its dimensions
        shape_msgs::SolidPrimitive primitive_obstacle;
        primitive_obstacle.type = primitive_obstacle.CYLINDER;
        primitive_obstacle.dimensions.resize(2);
        
        // Enlargment
        primitive_obstacle.dimensions[0] = obstacle_heigth_z + enlargment_obstacle_height; // Height
        primitive_obstacle.dimensions[1] = results->obs_size.at(j) + enlargment_obstacle_radius; // Radius

        // Define the pose of the obstacles
        geometry_msgs::Pose obstacle_object_pose;
        obstacle_object_pose.position.x = results->obs_pose.at(j).position.x;
        obstacle_object_pose.position.y = results->obs_pose.at(j).position.y;
        obstacle_object_pose.position.z = obstacle_pose_z;
        obstacle_object_pose.orientation = results->obs_pose.at(j).orientation;
        obstacle_object_pose = this->fixPose(obstacle_object_pose);

        collision_objects[i].primitives.push_back(primitive_obstacle);
        collision_objects[i].primitive_poses.push_back(obstacle_object_pose);
        collision_objects[i].operation = collision_objects[i].ADD;

        // Now, add the collision objects into the world
        this->planning_scene_interface.addCollisionObjects(collision_objects);

        ros::Publisher planning_scene_diff_publisher;
        planning_scene_diff_publisher = this->nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    }

}

void NodeA::detectionGoalEnded() {

    ROS_INFO("Detection finished successfully!");

    setCollisions();


}

void NodeA::pickPlaceActive(){}

void NodeA::pickPlaceEnded() {
    ROS_INFO("PickPlace action ended");
}

void NodeA::pickPlaceFeedback(const assignment2::PickPlaceFeedbackConstPtr& feedback) {}

void NodeA::detectionActiveGoal() {}

void NodeA::detectionFeedbackCallBack(const assignment2::DetectionFeedbackConstPtr& feedback) {
    ROS_INFO("Status: %s", feedback->status.c_str());
}

void NodeA::detectCylinderTables(const navigation::CustomResultConstPtr results) {
    float height = 0.69;
    float radius = 0.21;

    // First clean the planning scene
    std::vector <std::string> a = {"cylinder_table_0", "cylinder_table_1", "cylinder_table_2"};
    this->planning_scene_interface.removeCollisionObjects(a);

    // Add all the cylinders to the planning scene
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    for (int j = 0; j < results->obstacles.size(); j++) {
        printf("Center of the table: %f, %f\n", (results->obstacles.at(j)).x, (results->obstacles.at(j)).y);
        // Set the position and dimensions of the cylinder
        geometry_msgs::Pose pose;
        pose.position.x = (results->obstacles.at(j)).x + 0.25;
        pose.position.y = (results->obstacles.at(j)).y + j * 0.125;
        pose.position.z = height / 2.0;

        shape_msgs::SolidPrimitive cylinder;
        cylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
        cylinder.dimensions.resize(2);
        cylinder.dimensions[0] = height + 0.1;   // height
        cylinder.dimensions[1] = radius + 0.06;   // radius

        moveit_msgs::CollisionObject collision_object;

        collision_object.id = "cylinder_table_" + std::to_string(j);
        collision_object.header.frame_id = this->group_arm.getPlanningFrame();
        collision_object.primitives.push_back(cylinder);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_objects[0].ADD;

        collision_objects.push_back(collision_object);
    }
    // Add the collision object to the planning scene
    planning_scene_interface.addCollisionObjects(collision_objects);
    ros::Publisher planning_scene_diff_publisher;
    planning_scene_diff_publisher = this->nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    for (int j = 0; j < results->obstacles.size(); j++) {
        // Set the position and dimensions of the cylinder
        geometry_msgs::Pose pose;
        pose.position.x = (results->obstacles.at(j)).x + 0.25;
        pose.position.y = (results->obstacles.at(j)).y + j * 0.125;
        pose.position.z = height / 2.0;

        // Moving in front of the table
        navigation::CustomGoal movementGoal;
        movementGoal.x_goal = pose.position.x;
        movementGoal.y_goal = pose.position.y + 0.7;
        movementGoal.orientation = -90.0;
        this->ac_.sendGoal(movementGoal);
        this->ac_.waitForResult();

        ros::Duration(2.0).sleep();

        // Inspecting the camera
        ros::spinOnce();
        cv::Mat image = this->cv_ptr->image;

        // Computing the avg value of red, green and blue channels
        cv::Scalar avg_color = cv::mean(image);
        float red = avg_color[2];
        float green = avg_color[1];
        float blue = avg_color[0];

        if (red > green && red > blue) {
            ROS_INFO("RED");
            this->cylinders_map[3] = pose;
        } else if (green > red && green > blue) {
            ROS_INFO("GREEN");
            this->cylinders_map[2] = pose;
        } else {
            ROS_INFO("BLUE");
            this->cylinders_map[1] = pose;
        }
    }
}

void NodeA::navigateToTarget(std::vector<int> ids) {

    navigation::CustomGoal movementGoal;
    assignment2::DetectionGoal goalForDetection;

    // move to waypoint
    movementGoal.x_goal = 9;
    movementGoal.y_goal = 0;
    movementGoal.orientation = 0.0;
    this->ac_.sendGoal(movementGoal);
    this->ac_.waitForResult();

    for (int i = 0; i < ids.size(); i++) {

        // move to waypoint
        movementGoal.x_goal = 9;
        movementGoal.y_goal = 0;
        movementGoal.orientation = -90.0;
        this->ac_.sendGoal(movementGoal);
        this->ac_.waitForResult();

        if (ids.at(i) == 1) { //blue
            movementGoal.x_goal = 8.27;
            movementGoal.y_goal = -1.94;
            movementGoal.orientation = -90.0;
            goalForDetection.joint1 = -0.55;
            goalForDetection.joint2 = -0.60;
        } else if (ids.at(i) == 2) { //green
            // reaching safe point to safely reach then the pick position
            movementGoal.x_goal = 9;
            movementGoal.y_goal = -4.01;
            movementGoal.orientation = -180.0;
            this->ac_.sendGoal(movementGoal);
            this->ac_.waitForResult();
            movementGoal.x_goal = 7.45;
            movementGoal.y_goal = -4.01;
            movementGoal.orientation = 90.0;
            goalForDetection.joint1 = -0.40;
            goalForDetection.joint2 = -0.60;
        } else { //red
            movementGoal.x_goal = 7.5;
            movementGoal.y_goal = -1.965;
            movementGoal.orientation = -90.0;
            goalForDetection.joint1 = 0.45;
            goalForDetection.joint2 = -0.65;
        }

        // Send the goal to the server_node
        ROS_INFO("Reaching position to pick object %i", ids.at(i));
        goalForDetection.marker_ID = ids.at(i);
        this->ac_.sendGoal(movementGoal);
        this->ac_.waitForResult();
        this->bc_.sendGoal(goalForDetection,
                     boost::bind(&NodeA::detectionGoalEnded, this),
                     boost::bind(&NodeA::detectionActiveGoal, this),
                     boost::bind(&NodeA::detectionFeedbackCallBack, this, _1)); 
        this->bc_.waitForResult();

        // PICK must have already be done
        // Setting up the action goal for nodeC
        assignment2::PickPlaceGoal goal;
        goal.marker_pose = this->current_marker_pose;
        goal.marker_pose.position.z += 0.1; // 10cm above the object
        goal.marker_ID = this->current_marker_ID;
        goal.marker_size = this->current_marker_size;
        goal.pickOrPlace = true;

        ROS_INFO("SENDING GOAL TO C");
        // Sending it
        this->cc_.sendGoal(goal,
                    boost::bind(&NodeA::pickPlaceEnded, this),
                    boost::bind(&NodeA::pickPlaceActive, this),
                    boost::bind(&NodeA::pickPlaceFeedback, this, _1));
        this->cc_.waitForResult();

        if (ids.at(i) == 2) { //green
            // turning around to avoid the table
            movementGoal.orientation = -90;
            this->ac_.sendGoal(movementGoal);
            this->ac_.waitForResult();
            // reaching a safe point in order to safely reach then the waypoint
            movementGoal.x_goal = 9.0;
            movementGoal.y_goal = -4.01;
            this->ac_.sendGoal(movementGoal);
            this->ac_.waitForResult();
        }

        // go to waypoint
        movementGoal.x_goal = 9.0;
        movementGoal.y_goal = 0.0;
        movementGoal.orientation = 90.0;
        this->ac_.sendGoal(movementGoal);
        this->ac_.waitForResult();

        // go to waypoint
        movementGoal.x_goal = 10.0; //top finora: 10.0
        movementGoal.y_goal = 0.8; //top finora: 0.8
        movementGoal.orientation = -30.0; //top finora: -30.0
        this->ac_.sendGoal(movementGoal);
        this->ac_.waitForResult();
        ros::Duration(1.0).sleep();
        auto results = ac_.getResult();
        ROS_INFO("Found %ld tables", results->obstacles.size());

        // In results->obstacles there is an array with 3 positions
        // 0: RED, 1: GREEN, 2: BLUE
        // Move to the specified goal position
        int cylender_table_index = 0;
        if(ids.at(i) == 1) {
            cylender_table_index = 2;
        } else if(ids.at(i) == 2) {
            cylender_table_index = 1;
        }

        float height = 0.69;
        float radius = 0.21;
        // If we have not already detected the cylinders
        // we need to do it now once
        if (!this->detected_cylinders) {
            this->detectCylinderTables(results);
            this->detected_cylinders = true;
        }

        // Move in front of the table
        movementGoal.x_goal = (results->obstacles.at(cylender_table_index)).x+0.25;
        movementGoal.y_goal = (results->obstacles.at(cylender_table_index)).y - i*0.125 + 0.8;
        movementGoal.orientation = -90.0;
        this->ac_.sendGoal(movementGoal);
        this->ac_.waitForResult();


        geometry_msgs::Pose pose;
        pose.position.x = this->cylinders_map[ids.at(i)].position.x;
        pose.position.y = this->cylinders_map[ids.at(i)].position.y;
        pose.position.z = height + 0.1;

        // Convert the pose from "map" to "base_footprint"
        pose = fixPoseInverse(pose);

        // Setting up the action goal for nodeC
        goal.marker_pose.position.x = pose.position.x;
        goal.marker_pose.position.y = pose.position.y;
        goal.marker_pose.position.z = pose.position.z;
        goal.pickOrPlace = false;
        ROS_INFO("SENDING GOAL TO C TO PLACE OBJECT");
        // Sending it
        this->cc_.sendGoal(goal);
        this->cc_.waitForResult();

    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;

    NodeA node_a(nh);

    // Send a request to human_node and call the Action Server
    node_a.sendHumanNodeRequest();

    return 0;
}