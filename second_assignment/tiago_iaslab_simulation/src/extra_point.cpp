#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include "tiago_iaslab_simulation/pick_info.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <moveit/move_group_interface/move_group_interface.h>
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

bool get_poses = false;

bool goal_reached = false;

float objects[7][3] = {}; // 7 objects with x,y,z each in changed reference frame
int ids[3] = {};		  // order of 3 ids

std::vector<moveit_msgs::CollisionObject> collision_objects;

std::vector<moveit_msgs::CollisionObject> addCollisionObjects()
{

	collision_objects.resize(8);

	// Add the first table where the cube will originally be kept.
	collision_objects[0].id = "table1";
	collision_objects[0].header.frame_id = "map";

	collision_objects[0].primitives.resize(1);
	collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
	collision_objects[0].primitives[0].dimensions.resize(3);
	collision_objects[0].primitives[0].dimensions[0] = 1.1;
	collision_objects[0].primitives[0].dimensions[1] = 1.1;
	collision_objects[0].primitives[0].dimensions[2] = 0.82;

	collision_objects[0].primitive_poses.resize(1);
	collision_objects[0].primitive_poses[0].position.x = 7.75551;
	collision_objects[0].primitive_poses[0].position.y = -2.94581;
	collision_objects[0].primitive_poses[0].position.z = 0.3875;

	collision_objects[0].operation = collision_objects[0].ADD;

	////////////////////////////////////////////////////////////

	collision_objects[1].id = "blue";
	collision_objects[1].header.frame_id = "map";

	collision_objects[1].primitives.resize(1);
	collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].CYLINDER;
	collision_objects[1].primitives[0].dimensions.resize(3);
	collision_objects[1].primitives[0].dimensions[0] = objects[0][2] - 0.775;
	collision_objects[1].primitives[0].dimensions[1] = 0.0275;

	collision_objects[1].primitive_poses.resize(1);
	collision_objects[1].primitive_poses[0].position.x = objects[0][0];
	collision_objects[1].primitive_poses[0].position.y = objects[0][1];
	collision_objects[1].primitive_poses[0].position.z = objects[0][2] - (collision_objects[1].primitives[0].dimensions[0] / 2); //(objects[0][2]-0.775)/2;

	collision_objects[1].operation = collision_objects[1].ADD;

	////////////////////////////////////////////////////////////

	collision_objects[2].id = "green";
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

	collision_objects[3].id = "red";
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

// funziona
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
void moveToPos(float x, float y, float z, bool rotation)
{

	geometry_msgs::PoseStamped goal_pose;
	goal_pose.header.frame_id = "base_footprint";

	goal_pose.pose.position.x = x; // x object's coordinate
	goal_pose.pose.position.y = y; // y object's coordinate
	goal_pose.pose.position.z = z; // z object's coordinate + some space to grab it from above

	if (rotation)
	{
		// goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.011, 1.57, 0.037);//to pick it from vertical direction
	}

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
		throw std::runtime_error("No plan found");

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

float tables_pos[3][2] = {}; // 3 tables with x,y each

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
		ROS_INFO("Goal just went active");
	}
	void feedbackCb(const tiago_iaslab_simulation::ObstacleDetectionFeedbackConstPtr &feedback)
	{
		ROS_INFO("%s", feedback->status.c_str()); // print feedbacks
	}

private:
	Client ac;
};

////////////////////////////////////////////////////////////////////////////////////////

// EXTRA POINT:
// Scan laser data
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{

	// Object detection parameters
	const float detection_threshold = 0.1; // Min difference between consecutive scans to be classified as potential cilindric object
	const int min_consecutive_ranges = 10; // Minimum consecutive ranges to classify as an object

	bool is_start = false; // detected start of movable object (we want to detect the end of an object, its last scan)
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
				{
					is_start = true;
					starting_object_scan = i; // to detect the length of the object
				}
			}
			else
			{ // here the object ends
				int object_scans = i - starting_object_scan;
				ROS_INFO("object_scans = %d", object_scans);
				if ((object_scans > min_consecutive_ranges) && (object_scans < 100))
				{ // detect only our cilindric objects

					// this is a circular object -> object detected
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

					tables_pos[table_count][0] = x;
					tables_pos[table_count][1] = y;
					table_count++;
				}
				is_start = false;
			}
		}
	}
	callbackExecuted = true;
}

//////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{

	ros::init(argc, argv, "pick_and_place");
	ros::NodeHandle nh;

	objects[0][0] = 8.03168;
	objects[0][1] = -2.64195;
	objects[0][2] = 0.877631;
	objects[1][0] = 7.69315;
	objects[1][1] = -3.31828;
	objects[1][2] = 0.796102;
	objects[2][0] = 7.45091;
	objects[2][1] = -2.65624;
	objects[2][2] = 0.824447;
	objects[3][0] = 7.64161;
	objects[3][1] = -2.63838;
	objects[3][2] = 0.977064;
	objects[4][0] = 7.92755;
	objects[4][1] = -3.26637;
	objects[4][2] = 0.978085;
	objects[5][0] = 7.56808;
	objects[5][1] = -3.17509;
	objects[5][2] = 0.976828;
	objects[6][0] = 7.83662;
	objects[6][1] = -2.64006;
	objects[6][2] = 0.977333;

	// create our planning scene and define collision objects
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// add collision objects
	collision_objects = addCollisionObjects();
	planning_scene_interface.applyCollisionObjects(collision_objects);
	ROS_INFO("Added collision objects to planning_scene");

	return 0;
}
