#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "tiago_iaslab_simulation/pick_info.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <vector>
#include <map>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tiago_iaslab_simulation/ObstacleDetectionAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/LaserScan.h"

bool get_poses = false;	   // became true when we get coordinates from nodeB, to exit the spin
bool goal_reached = false; // became true when a certain position is reached, to exit the spin

// let's define arrays that store objects' coordinates and ids order
float objects[7][3] = {}; // 7 objects with x,y,z each
int ids[3] = {};		  // order of 3 ids

// array used to define collision objects
std::vector<moveit_msgs::CollisionObject> collision_objects;

// function that reates collision objects
std::vector<moveit_msgs::CollisionObject> addCollisionObjects()
{
	collision_objects.resize(8);

	// Add the first table where the cube will originally be kept.
	collision_objects[0].id = "table1";
	collision_objects[0].header.frame_id = "map"; // we're using map reference frame

	collision_objects[0].primitives.resize(1);
	collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
	collision_objects[0].primitives[0].dimensions.resize(3);
	collision_objects[0].primitives[0].dimensions[0] = 1.1;
	collision_objects[0].primitives[0].dimensions[1] = 1.1;
	collision_objects[0].primitives[0].dimensions[2] = 0.82;

	collision_objects[0].primitive_poses.resize(1);
	collision_objects[0].primitive_poses[0].position.x = 7.75551;
	collision_objects[0].primitive_poses[0].position.y = -2.94581;
	collision_objects[0].primitive_poses[0].position.z = 0.41;

	collision_objects[0].operation = collision_objects[0].ADD;

	////////////////////////////////////////////////////////////

	collision_objects[1].id = "blue"; // BLUE HEXAGON
	collision_objects[1].header.frame_id = "map";

	collision_objects[1].primitives.resize(1);
	collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].CYLINDER;
	collision_objects[1].primitives[0].dimensions.resize(3);
	collision_objects[1].primitives[0].dimensions[0] = objects[0][2] - 0.775;
	collision_objects[1].primitives[0].dimensions[1] = 0.0275;

	collision_objects[1].primitive_poses.resize(1);
	collision_objects[1].primitive_poses[0].position.x = objects[0][0];
	collision_objects[1].primitive_poses[0].position.y = objects[0][1];
	collision_objects[1].primitive_poses[0].position.z = objects[0][2] - (collision_objects[1].primitives[0].dimensions[0] / 2);

	collision_objects[1].operation = collision_objects[1].ADD;

	////////////////////////////////////////////////////////////

	collision_objects[2].id = "green"; // GREEN TRIANGLE
	collision_objects[2].header.frame_id = "map";

	collision_objects[2].primitives.resize(1);
	collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
	collision_objects[2].primitives[0].dimensions.resize(3);
	collision_objects[2].primitives[0].dimensions[0] = 0.055;
	collision_objects[2].primitives[0].dimensions[1] = 0.055;
	collision_objects[2].primitives[0].dimensions[2] = 0.055;

	collision_objects[2].primitive_poses.resize(1);
	collision_objects[2].primitive_poses[0].position.x = objects[1][0];
	collision_objects[2].primitive_poses[0].position.y = objects[1][1];
	collision_objects[2].primitive_poses[0].position.z = objects[1][2];

	collision_objects[2].operation = collision_objects[2].ADD;

	////////////////////////////////////////////////////////////

	collision_objects[3].id = "red"; // RED CUBE
	collision_objects[3].header.frame_id = "map";

	collision_objects[3].primitives.resize(1);
	collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
	collision_objects[3].primitives[0].dimensions.resize(3);
	collision_objects[3].primitives[0].dimensions[0] = 0.055;
	collision_objects[3].primitives[0].dimensions[1] = 0.055;
	collision_objects[3].primitives[0].dimensions[2] = 0.055;

	collision_objects[3].primitive_poses.resize(1);
	collision_objects[3].primitive_poses[0].position.x = objects[2][0];
	collision_objects[3].primitive_poses[0].position.y = objects[2][1];
	collision_objects[3].primitive_poses[0].position.z = objects[2][2];

	collision_objects[3].operation = collision_objects[3].ADD;

	////////////////////////////////////////////////////////////

	collision_objects[4].id = "gold0";
	collision_objects[4].header.frame_id = "map";

	collision_objects[4].primitives.resize(1);
	collision_objects[4].primitives[0].type = collision_objects[4].primitives[0].CYLINDER;
	collision_objects[4].primitives[0].dimensions.resize(3);
	collision_objects[4].primitives[0].dimensions[0] = 0.3;
	collision_objects[4].primitives[0].dimensions[1] = 0.075;

	collision_objects[4].primitive_poses.resize(1);
	collision_objects[4].primitive_poses[0].position.x = objects[3][0];
	collision_objects[4].primitive_poses[0].position.y = objects[3][1];
	collision_objects[4].primitive_poses[0].position.z = 0.82 + 0.15;

	collision_objects[4].operation = collision_objects[4].ADD;

	////////////////////////////////////////////////////////////

	collision_objects[5].id = "gold1";
	collision_objects[5].header.frame_id = "map";

	collision_objects[5].primitives.resize(1);
	collision_objects[5].primitives[0].type = collision_objects[5].primitives[0].CYLINDER;
	collision_objects[5].primitives[0].dimensions.resize(3);
	collision_objects[5].primitives[0].dimensions[0] = 0.3;
	collision_objects[5].primitives[0].dimensions[1] = 0.075;

	collision_objects[5].primitive_poses.resize(1);
	collision_objects[5].primitive_poses[0].position.x = objects[4][0];
	collision_objects[5].primitive_poses[0].position.y = objects[4][1];
	collision_objects[5].primitive_poses[0].position.z = 0.82 + 0.15;

	collision_objects[5].operation = collision_objects[5].ADD;

	////////////////////////////////////////////////////////////

	collision_objects[6].id = "gold2";
	collision_objects[6].header.frame_id = "map";

	collision_objects[6].primitives.resize(1);
	collision_objects[6].primitives[0].type = collision_objects[6].primitives[0].CYLINDER;
	collision_objects[6].primitives[0].dimensions.resize(3);
	collision_objects[6].primitives[0].dimensions[0] = 0.3;
	collision_objects[6].primitives[0].dimensions[1] = 0.075;

	collision_objects[6].primitive_poses.resize(1);
	collision_objects[6].primitive_poses[0].position.x = objects[5][0];
	collision_objects[6].primitive_poses[0].position.y = objects[5][1];
	collision_objects[6].primitive_poses[0].position.z = 0.82 + 0.15;

	collision_objects[6].operation = collision_objects[6].ADD;

	////////////////////////////////////////////////////////////

	collision_objects[7].id = "gold3";
	collision_objects[7].header.frame_id = "map";

	collision_objects[7].primitives.resize(1);
	collision_objects[7].primitives[0].type = collision_objects[7].primitives[0].CYLINDER;
	collision_objects[7].primitives[0].dimensions.resize(3);
	collision_objects[7].primitives[0].dimensions[0] = 0.3;
	collision_objects[7].primitives[0].dimensions[1] = 0.075;

	collision_objects[7].primitive_poses.resize(1);
	collision_objects[7].primitive_poses[0].position.x = objects[6][0];
	collision_objects[7].primitive_poses[0].position.y = objects[6][1];
	collision_objects[7].primitive_poses[0].position.z = 0.82 + 0.15;

	collision_objects[7].operation = collision_objects[7].ADD;

	// planning_scene_interface.applyCollisionObjects(collision_objects);
	return collision_objects;
}

// function that attach 2 joints, used to attach our objects to the end effector
void attachObjects(std::string name, std::string link)
{
	// Use ROS ServiceClient to call /link_attacher_node/attach service
	// attach object to robot's link
	ros::NodeHandle nh;
	ros::ServiceClient attachClient = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");

	// Create a service message
	gazebo_ros_link_attacher::Attach attachSrv;

	// Set the parameters for attaching objects
	attachSrv.request.model_name_1 = "tiago";
	attachSrv.request.link_name_1 = "gripper_right_finger_link";
	attachSrv.request.model_name_2 = name;
	attachSrv.request.link_name_2 = link;

	// Call the service
	if (attachClient.call(attachSrv))
	{
		ROS_INFO("Objects attached successfully");
	}
	else
	{
		ROS_ERROR("Failed to attach objects");
		return;
	}
}

// function that detach 2 joints, used to place our objects to the destination table
void detachObjects(std::string name, std::string link)
{
	// Use ROS ServiceClient to call /link_attacher_node/detach service
	// detach object from robot's link
	ros::NodeHandle nh;
	ros::ServiceClient detachClient = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

	// Create a service message
	gazebo_ros_link_attacher::Attach detachSrv;

	// Set the parameters for detaching objects
	detachSrv.request.model_name_1 = "tiago";
	detachSrv.request.link_name_1 = "gripper_right_finger_link";
	detachSrv.request.model_name_2 = name;
	detachSrv.request.link_name_2 = link;

	// Call the service
	if (detachClient.call(detachSrv))
	{
		ROS_INFO("Objects detached successfully");
	}
	else
	{
		ROS_ERROR("Failed to detach objects");
		return;
	}
}

// function that moves the end effector to a pose defined in base_footprint reference frame
void pick(float x, float y, float z)
{

	geometry_msgs::PoseStamped goal_pose;
	goal_pose.header.frame_id = "base_footprint";

	goal_pose.pose.position.x = x; // x object's coordinate
	goal_pose.pose.position.y = y; // y object's coordinate
	goal_pose.pose.position.z = z; // z object's coordinate + some space to grab it from above

	// if(rotation){
	// goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.011, 1.57, 0.037);//to pick it from vertical direction
	//}

	goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI / 2.0, 0.0); // Vertical orientation

	ros::AsyncSpinner spinner(1);
	spinner.start();

	std::vector<std::string> torso_arm_joint_names;
	// Select group of joints
	moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
	// Choose our planner

	// move_group.setGoalOrientationTolerance(0.1);  // Set your orientation tolerance
	// move_group.setGoalPositionTolerance(0.1);     // Set your position tolerance

	// group_arm_torso.setPlannerId("SBLkConfigDefault");
	group_arm_torso.setPoseReferenceFrame("base_footprint");
	group_arm_torso.setPoseTarget(goal_pose);

	ROS_INFO_STREAM("Planning to move " << group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " << group_arm_torso.getPlanningFrame());

	group_arm_torso.setStartStateToCurrentState();
	group_arm_torso.setMaxVelocityScalingFactor(1.0);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	// Set maximum time to find a plan
	group_arm_torso.setPlanningTime(20.0);
	bool success = bool(group_arm_torso.plan(my_plan));

	if (!success)
	{
		// throw std::runtime_error("No plan found");
		ROS_INFO("No plan found, retrying...");
		pick(x, y, z);
	}

	ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

	// Execute the plan
	ros::Time start = ros::Time::now();

	moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
	if (!bool(e))
		throw std::runtime_error("Error executing plan");

	ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

	spinner.stop();
}

//////////////////////////////////////////////////////////////////////////////////////////////////

// PART ABOUT MOVING ROBOT TO PLACE POSITION:

typedef actionlib::SimpleActionClient<tiago_iaslab_simulation::ObstacleDetectionAction> Client;

bool callbackExecuted = false;

// array that stores coordinates of 3 tables detected using laserscan
float tables_pos[3][2] = {}; // 3 tables with x,y each
//[0] = rosso, [1] = verde, [2] = blu

// usual class to move our robot to a goal pose
class ServiceClient // classe per muovere il robot to a target position (per farlo arrivare dalla parte opposta del tavolo)
{
public:
	ServiceClient() : ac("fibonacci", true)
	{
		ROS_INFO("Waiting for action server to start.");
		ac.waitForServer();
		ROS_INFO("Action server started, sending goal.");
	}

	void send_goal(float x, float y, float z) // reaches pose_b and detects cilindric objects
	{

		tiago_iaslab_simulation::ObstacleDetectionGoal goal;
		float goal_pose[3] = {x, y, z}; // vector of 3 float for x position, y position and orientation

		goal.pose_b.push_back(x);
		goal.pose_b.push_back(y);
		goal.pose_b.push_back(z);

		ac.sendGoal(goal,
					boost::bind(&ServiceClient::doneCb, this, _1, _2),
					boost::bind(&ServiceClient::activeCb, this),
					boost::bind(&ServiceClient::feedbackCb, this, _1));
	}

	void doneCb(const actionlib::SimpleClientGoalState &state,
				const tiago_iaslab_simulation::ObstacleDetectionResultConstPtr &result)
	{
		goal_reached = true;
	}
	void activeCb()
	{
		// ROS_INFO("Goal just went active");
	}
	void feedbackCb(const tiago_iaslab_simulation::ObstacleDetectionFeedbackConstPtr &feedback)
	{
		ROS_INFO("%s", feedback->status.c_str()); // print feedbacks
	}

private:
	Client ac;
};

////////////////////////////////////////////////////////////////////////////////////////

// EXTRA POINT IMPLEMENTATION:
// Scan laser data to detect exact tables' positions
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
	// Create a TransformListener to transform between coordinates frames
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);
	ros::WallDuration(5.0).sleep(); // wait time to generate tf

	// Object detection parameters
	const float detection_threshold = 0.1; // Min difference between consecutive scans to be classified as potential cilindric object
	const int min_consecutive_ranges = 10; // Minimum consecutive ranges to classify as an object

	bool is_start = false; // detected start of object (we want to detect the end of an object, its last scan)
	int starting_object_scan = 0;

	int table_count = 0; // 1,2 or 3

	for (size_t i = 150; i < scan_msg->ranges.size() - 22; ++i)
	{ // scan the ranges array
		float range = scan_msg->ranges[i];

		float prev_range = scan_msg->ranges[i - 1];
		float next_range = scan_msg->ranges[i + 1];

		// Calculate percentage difference between consecutive scans
		float diff_percentage = std::abs(range - prev_range) / prev_range;

		// Check if the percentage difference is above the threshold
		if ((diff_percentage > detection_threshold) && (diff_percentage < 8.0))
		{ // there is a big difference on 2 consecudive scans

			if (is_start == false)
			{ // here the object starts
				if (range < prev_range)
				{ // table are near to the robot than walls
					is_start = true;
					starting_object_scan = i; // to detect the length of the object
				}
			}
			else
			{
				int object_scans = i - starting_object_scan;
				if ((object_scans > min_consecutive_ranges) && (object_scans < 100))
				{
					// this is a circular object -> table detected
					int object_center = round(i - (object_scans / 2)); // scan at center of cilinder
					double angle = scan_msg->angle_min + object_center * scan_msg->angle_increment;
					float x = prev_range * cos(angle); // polar to cartesan coordinates
					float y = prev_range * sin(angle);
					ROS_INFO("Table detected at x= %f, y= %f", x, y);

					if (table_count == 3)
					{
						ROS_INFO("Error: detected more than 3 table!");
						return;
					}

					// convert TO MAP FRAME and save values in table_pos
					// convert object frame from map to base_footprint to pick it
					geometry_msgs::PoseStamped robot_pose;
					robot_pose.header.frame_id = "base_footprint";
					robot_pose.pose.position.x = x;
					robot_pose.pose.position.y = y;
					robot_pose.pose.position.z = 0;

					std::string target_frame = "map";
					geometry_msgs::PoseStamped map_pose;

					try
					{
						geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(target_frame, robot_pose.header.frame_id, ros::Time::now(), ros::Duration(3.0));
						tf2::doTransform(robot_pose, map_pose, transformStamped);
					}
					catch (tf2::TransformException &ex)
					{
						ROS_ERROR("%s", ex.what());
						return;
					}
					//----

					tables_pos[table_count][0] = map_pose.pose.position.x;
					tables_pos[table_count][1] = map_pose.pose.position.y;
					table_count++;
				}
				is_start = false;
			}
		}
	}
	callbackExecuted = true;
}

//////////////////////////////////////////////////////////////

// callback called when nodeC recives the message from nodeB containing the coordinates of the detected objects
void messageCallback(const tiago_iaslab_simulation::pick_info::ConstPtr &msg)
{
	ROS_INFO("Message recived");

	// print id order
	ROS_INFO("This is the order that the human tell us:");
	for (int i = 0; i < msg->id_order.size(); ++i)
	{
		std::cout << "ID " << msg->id_order[i] << std::endl;
	}

	// print objects positions
	ROS_INFO("Poses recived from node_B:");
	std::cout << "Object " << 1 << ": x = " << msg->obj1_pos[0] << ", y = " << msg->obj1_pos[1] << ", z = " << msg->obj1_pos[2] << std::endl;
	std::cout << "Object " << 2 << ": x = " << msg->obj2_pos[0] << ", y = " << msg->obj2_pos[1] << ", z = " << msg->obj2_pos[2] << std::endl;
	std::cout << "Object " << 3 << ": x = " << msg->obj3_pos[0] << ", y = " << msg->obj3_pos[1] << ", z = " << msg->obj3_pos[2] << std::endl;
	std::cout << "Object " << 4 << ": x = " << msg->obj4_pos[0] << ", y = " << msg->obj4_pos[1] << ", z = " << msg->obj4_pos[2] << std::endl;
	std::cout << "Object " << 5 << ": x = " << msg->obj5_pos[0] << ", y = " << msg->obj5_pos[1] << ", z = " << msg->obj5_pos[2] << std::endl;
	std::cout << "Object " << 6 << ": x = " << msg->obj6_pos[0] << ", y = " << msg->obj6_pos[1] << ", z = " << msg->obj6_pos[2] << std::endl;
	std::cout << "Object " << 7 << ": x = " << msg->obj7_pos[0] << ", y = " << msg->obj7_pos[1] << ", z = " << msg->obj7_pos[2] << std::endl;

	// fill our objects array with the recived values
	objects[0][0] = msg->obj1_pos[0];
	objects[0][1] = msg->obj1_pos[1];
	objects[0][2] = msg->obj1_pos[2];
	objects[1][0] = msg->obj2_pos[0];
	objects[1][1] = msg->obj2_pos[1];
	objects[1][2] = msg->obj2_pos[2];
	objects[2][0] = msg->obj3_pos[0];
	objects[2][1] = msg->obj3_pos[1];
	objects[2][2] = msg->obj3_pos[2];
	objects[3][0] = msg->obj4_pos[0];
	objects[3][1] = msg->obj4_pos[1];
	objects[3][2] = msg->obj4_pos[2];
	objects[4][0] = msg->obj5_pos[0];
	objects[4][1] = msg->obj5_pos[1];
	objects[4][2] = msg->obj5_pos[2];
	objects[5][0] = msg->obj6_pos[0];
	objects[5][1] = msg->obj6_pos[1];
	objects[5][2] = msg->obj6_pos[2];
	objects[6][0] = msg->obj7_pos[0];
	objects[6][1] = msg->obj7_pos[1];
	objects[6][2] = msg->obj7_pos[2];

	// do the same with the id order array
	ids[0] = msg->id_order[0];
	ids[1] = msg->id_order[1];
	ids[2] = msg->id_order[2];

	get_poses = true;
}

///////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{

	ros::init(argc, argv, "pick_and_place");
	ros::NodeHandle nh;

	// Create a TransformListener to transform between coordinates frames
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);

	// subscribe to message to read detected objects from nodeB
	ros::Subscriber sub = nh.subscribe("message", 1, messageCallback);
	while ((ros::ok()) && (!get_poses))
	{
		ros::Duration(2.5).sleep();
		ros::spinOnce();
	}
	sub.shutdown(); // stop listening the messages

	// create our planning scene and define collision objects
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// add collision objects
	collision_objects = addCollisionObjects();
	planning_scene_interface.applyCollisionObjects(collision_objects);
	ROS_INFO("Added collision objects to planning_scene");

	ros::WallDuration(1.0).sleep();

	bool above_table = true; // true if we are in the left side of the table, false otherwise

	ServiceClient movement_client;

	// let's check the most convenient side of the table to pick first object
	float table_y_coord = -2.945;
	float obj_y_coord = objects[ids[0] - 1][1];
	// let's compare the y coordinates of the object and the center of the table
	if (obj_y_coord > table_y_coord)
	{ // we have to change side
		// we use 3 intermediate poses to avoid the table
		ROS_INFO("First object is on the other side of the table, let's move the robot there");

		movement_client.send_goal(8.8, -4.0, -M_PI / 2);
		while ((ros::ok()) && (!goal_reached))
		{
			ros::Duration(2.5).sleep();
			ros::spinOnce();
		}
		goal_reached = false; // restore variable

		movement_client.send_goal(8.8, -2.0, -M_PI / 2); // alto sx
		while ((ros::ok()) && (!goal_reached))
		{
			ros::Duration(2.5).sleep();
			ros::spinOnce();
		}
		goal_reached = false; // restore variable

		// movement_client.send_goal(7.8, -2.0, M_PI / 2);//posizione tavolo dall basso
		movement_client.send_goal(objects[ids[0] - 1][0], -2.0, M_PI / 2);
		while ((ros::ok()) && (!goal_reached))
		{
			ros::Duration(2.5).sleep();
			ros::spinOnce();
		}
		goal_reached = false; // restore variable

		above_table = false;
	}
	else
	{ // first object is in the left side of the table, let's go in front of it
		movement_client.send_goal(objects[ids[0] - 1][0], -3.9, -M_PI / 2);
		while ((ros::ok()) && (!goal_reached))
		{
			ros::Duration(2.5).sleep();
			ros::spinOnce();
		}
		goal_reached = false; // restore variable
	}

	for (int obj = 0; obj < 3; obj++)
	{ // cycle that iterates over the 3 objects we have to pick

		int current_obj = ids[obj] - 1; // position on objects array of our current object
		ROS_INFO("Object to pick: %d", ids[obj]);

		// remove current object from collsion objects
		collision_objects[ids[obj]].operation = collision_objects[ids[obj]].REMOVE;
		// Apply the updated vector of collision objects to the planning scene
		planning_scene_interface.applyCollisionObjects(collision_objects);

		// CONTROLLARE SE L'ARRAY COLLISIONOBJECTS VIENE RIDIMENSIONATO, SE NO ALL'OGGETTO SUCCESSIVO L'INDICE NON SARA' PIU' CORRETTO

		// convert object frame from map to base_footprint to pick it
		geometry_msgs::PoseStamped map_pose;
		map_pose.header.frame_id = "map";
		map_pose.pose.position.x = objects[current_obj][0];
		map_pose.pose.position.y = objects[current_obj][1];
		map_pose.pose.position.z = objects[current_obj][2];

		std::string target_frame = "base_footprint";
		geometry_msgs::PoseStamped robot_pose;

		try
		{
			// Wait for the transform from map frame to base_footprint frame
			geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(target_frame, map_pose.header.frame_id, ros::Time::now(), ros::Duration(3.0));

			// Transform the pose from map frame to base_footprint frame
			tf2::doTransform(map_pose, robot_pose, transformStamped);
		}
		catch (tf2::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			return 0;
		}

		// move arm above the object
		planning_scene_interface.applyCollisionObjects(collision_objects);
		if (obj == 0)
			pick(robot_pose.pose.position.x, robot_pose.pose.position.y, 1.15);
		else
			pick(robot_pose.pose.position.x, robot_pose.pose.position.y, 1.15);
		ROS_INFO("Picking object from table");

		ros::WallDuration(10.0).sleep();

		// Works for green (even if left out of the table) and blue (taken and left correctly) but does not find the plan to take the red
		// move the robot to the place position (in front of the 3 tables):

		// attach our object to the end effector of tiago
		if (current_obj == 0)
		{
			attachObjects("Hexagon", "Hexagon_link");
		}
		else if (current_obj == 1)
		{
			attachObjects("Triangle", "Triangle_link");
		}
		else
		{
			attachObjects("cube", "cube_link");
		}

		ros::WallDuration(2.0).sleep();

		// move the object vertically to a safer position
		planning_scene_interface.applyCollisionObjects(collision_objects);
		pick(0.5, robot_pose.pose.position.y, 1.2);

		ros::WallDuration(10.0).sleep();

		// move the robot to the place position (in front of the 3 tables):

		// move to a partial location to avoid table if necessary
		if (above_table == true)
		{ // we are above the table, we must avoid it
			ROS_INFO("We're in the opposite side of the table, let's reach 2 intermediate poses and then the 3 place tables");
			movement_client.send_goal(7.8, -4.0, M_PI / 8);
			while ((ros::ok()) && (!goal_reached))
			{
				ros::Duration(1.5).sleep();
				ros::spinOnce();
			}
			goal_reached = false; // restore variable

			movement_client.send_goal(8.8, -4.0, -M_PI / 2);
			while ((ros::ok()) && (!goal_reached))
			{
				ros::Duration(1.5).sleep();
				ros::spinOnce();
			}
			goal_reached = false; // restore variable

			movement_client.send_goal(8.8, -1.0, -M_PI / 2);
			while ((ros::ok()) && (!goal_reached))
			{
				ros::Duration(1.5).sleep();
				ros::spinOnce();
			}
			goal_reached = false; // restore variable
		}

		// move in front of the 3 tables
		movement_client.send_goal(11.5, 1.2, M_PI / 2);

		while ((ros::ok()) && (!goal_reached))
		{
			ros::Duration(2.0).sleep();
			ros::spinOnce();
		}
		goal_reached = false; // restore variable

		//////////////////////////Extra point
		ROS_INFO("Scanning laser data to detect tables' positions");
		// Scan laser data to detect tables poses
		ros::NodeHandle nh_;
		// Subscribe to the /scan topic
		ros::Subscriber scan_sub = nh_.subscribe("/scan", 1, scanCallback);

		// Spin to allow callback function to run
		while ((ros::ok()) && (!callbackExecuted))
		{
			ros::Duration(2.0).sleep();
			ros::spinOnce();
		}
		scan_sub.shutdown();

		// move in front of our table
		// 0=rosso, 1=verde, 2=blu
		int table = 0;
		if (current_obj == 0)
		{ // Blue object
			table = 2;
		}
		else if (current_obj == 1)
		{ // Green object
			table = 1;
		}
		else
		{ // Red object
			table = 0;
		}

		for (int y = 0; y < 3; y++)
		{
			ROS_INFO("table %d: x=%f  y=%f", y, tables_pos[y][0], tables_pos[y][1]);
		}

		// float x = 11.5 + tables_pos[table][1]; //take into account the rotation, from abs frame to rel
		// float y = 1.2 - tables_pos[table][0] + 0.4; //to place in front of it
		float x = tables_pos[table][0];
		float y = tables_pos[table][1] + 0.37; // to place in front of the table
		movement_client.send_goal(x, y, M_PI / 2);

		while ((ros::ok()) && (!goal_reached))
		{
			ros::Duration(2.0).sleep();
			ros::spinOnce();
		}
		goal_reached = false; // restore variable

		// place
		planning_scene_interface.applyCollisionObjects(collision_objects);

		// convert object frame from map to base_footprint to pick it
		// geometry_msgs::PoseStamped map_pose;
		map_pose.header.frame_id = "map";
		map_pose.pose.position.x = tables_pos[table][0];
		map_pose.pose.position.y = tables_pos[table][1];
		map_pose.pose.position.z = 0;

		target_frame = "base_footprint";
		// geometry_msgs::PoseStamped robot_pose;//gia dichiarato sopra

		try
		{
			// Wait for the transform from map frame to base_footprint frame
			geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(target_frame, map_pose.header.frame_id, ros::Time::now(), ros::Duration(3.0));

			// Transform the pose from map frame to base_footprint frame
			tf2::doTransform(map_pose, robot_pose, transformStamped);
		}
		catch (tf2::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			return 0;
		}

		ROS_INFO("placing object");
		pick(robot_pose.pose.position.x + 0.2, robot_pose.pose.position.y, 1.15);
		// pick(0.7, 0.0, 1.15, false);//place object on front of the robot over the table

		ros::WallDuration(10.0).sleep();

		// detach object from end effector
		if (current_obj == 0)
		{
			detachObjects("Hexagon", "Hexagon_link");
		}
		else if (current_obj == 1)
		{
			detachObjects("Triangle", "Triangle_link");
		}
		else
		{
			detachObjects("cube", "cube_link");
		}

		ros::WallDuration(2.0).sleep();

		// move end effector to a safer pose
		planning_scene_interface.applyCollisionObjects(collision_objects);
		pick(robot_pose.pose.position.x, robot_pose.pose.position.y, 1.2); // high
		pick(0.5, 0.0, 1.2);											   // closer to the robot

		ros::WallDuration(10.0).sleep();

		if (obj == 2)
		{
			break;
		} // program finishes at third object

		// RETURN TO THE PICKING TABLE
		ROS_INFO("Coming back to the pick table");
		ROS_INFO("Finding best side of the table according to next object's position");

		// let's check the most convenient side of the table to pick next object
		float table_y_coord = -2.945;
		int next_obj = ids[obj + 1] - 1; // y coordinate of next obj to pick
		float obj_y_coord = objects[next_obj][1];

		if (obj_y_coord > table_y_coord)
		{ // we have to reach table from below

			movement_client.send_goal(objects[next_obj][0], -2.0, M_PI / 2);
			while ((ros::ok()) && (!goal_reached))
			{
				ros::Duration(2.5).sleep();
				ros::spinOnce();
			}
			goal_reached = false; // restore variable

			above_table = false;
		}
		else
		{ // we have to reach the table from above
			// let's use intermediate poses
			movement_client.send_goal(8.8, -2.0, M_PI / 2);
			while ((ros::ok()) && (!goal_reached))
			{
				ros::Duration(2.5).sleep();
				ros::spinOnce();
			}
			goal_reached = false; // restore variable

			movement_client.send_goal(8.8, -4.0, M_PI / 2); // alto sx
			while ((ros::ok()) && (!goal_reached))
			{
				ros::Duration(2.5).sleep();
				ros::spinOnce();
			}
			goal_reached = false; // restore variable

			movement_client.send_goal(objects[next_obj][0], -3.9, -M_PI / 2); // Table position from above
			while ((ros::ok()) && (!goal_reached))
			{
				ros::Duration(2.5).sleep();
				ros::spinOnce();
			}
			goal_reached = false; // restore variable

			above_table = true;
		}
	}

	return 0;
}
