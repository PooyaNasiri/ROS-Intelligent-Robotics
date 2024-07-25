#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tiago_iaslab_simulation/ObstacleDetectionAction.h>
#include <tiago_iaslab_simulation/Objs.h>
#include "tiago_iaslab_simulation/pick_info.h"
#include "tiago_iaslab_simulation/start_nodeB.h"
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<tiago_iaslab_simulation::ObstacleDetectionAction> Client;

bool goal_reached = false;	   // became true when a certain position is reached, to exit the spin
bool nodeA_message = false;	   // became true when nodeB reaches the message from nodeA, to exit the spin
bool objects_detected = false; // became true when we detect some objects, to exit the spin

// array that stores coordinates (x,y,z) of 8 objects in the table. NodeB will fill this array with such values
float objects[7][3] = {}; // 7 objects with x,y,z each

// class that implements a service client, used to move the robot to a goal position
//  we use this class to reach the 3 intermediate poses to detect the objects from a different point of view
class ServiceClient
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

// callback function of tag_detections topic, saves detected values to our array using map reference frame
void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{

	ROS_INFO("Received %lu AprilTag detections", msg->detections.size());

	// Define the pose in the camera frame
	geometry_msgs::PoseStamped camera_pose;
	camera_pose.header.frame_id = "xtion_rgb_optical_frame";

	// Create a TransformListener
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);

	std::string target_frame = "map";
	geometry_msgs::PoseStamped map_pose;

	for (const auto &detection : msg->detections)
	{
		ROS_INFO("AprilTag ID: %d", detection.id[0]);
		ROS_INFO("Position (x, y, z): %f, %f, %f",
				 detection.pose.pose.pose.position.x,
				 detection.pose.pose.pose.position.y,
				 detection.pose.pose.pose.position.z);
		int id = detection.id[0] - 1;

		if (objects[id][0] == 0.0)
		{ // if not already detected
			// Let's transform from camera frame to map (global) frame
			camera_pose.pose.position.x = detection.pose.pose.pose.position.x;
			camera_pose.pose.position.y = detection.pose.pose.pose.position.y;
			camera_pose.pose.position.z = detection.pose.pose.pose.position.z;
			try
			{
				// Wait for the transform from camera frame to robot frame
				geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(
					target_frame, camera_pose.header.frame_id, camera_pose.header.stamp, ros::Duration(3.0));

				// Transform the pose from camera frame to map frame
				tf2::doTransform(camera_pose, map_pose, transformStamped);
			}
			catch (tf2::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
			}
			// assign new transformed coordinates to our global vector of coordinates
			objects[id][0] = map_pose.pose.position.x;
			objects[id][1] = map_pose.pose.position.y;
			objects[id][2] = map_pose.pose.position.z;
		}
	}
	objects_detected = true; // stop the detection of objects for now
}

// recive message from nodeA: nodeB can start the detection
void messageCallback(const tiago_iaslab_simulation::start_nodeB::ConstPtr &msg)
{
	ROS_INFO("Message recived: nodeB starts");
	nodeA_message = true; // to stop the spin
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "objects_detection");
	ros::NodeHandle nh;

	// Create a TransformListener to convert coordinates between 2 reference frames
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);

	// get message from nodeA: nodeB can start the detection of objects
	ros::Subscriber sub = nh.subscribe("start_nodeB", 1, messageCallback);
	while ((ros::ok()) && (!nodeA_message))
	{
		ros::Duration(2.5).sleep();
		ros::spinOnce();
	}

	// human node gives us the order in which we have to pick the 3 objects, we save this order in id_order array
	// Create a service client for the "/human_objects_srv" service
	ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");

	// Create a service request
	tiago_iaslab_simulation::Objs srv;
	srv.request.ready = true;	 // Set to true to receive object IDs
	srv.request.all_objs = true; // Set to true if you want all object IDs

	int id_order[3] = {}; // specifies the order to pick the objects (id_order[0] is the first)
	// Call the service
	if (client.call(srv))
	{
		if (srv.response.ids.size() > 0)
		{
			ROS_INFO("Received object ID(s) from human_node:");
			int i = 0;
			for (int id : srv.response.ids)
			{
				ROS_INFO("%d", id);
				id_order[i] = id;
				i++;
			}
		}
		else
		{
			ROS_ERROR("No object IDs received from human_node");
		}
	}
	else
	{
		ROS_ERROR("Failed to call human_objects_srv");
		return 1;
	}

	// Subscribe to the tag_detections topic (that sends us apriltag's coordinates that is detecting at any moment)
	sub = nh.subscribe("tag_detections", 1, tagCallback);
	while ((ros::ok()) && (!objects_detected))
	{
		ros::Duration(2.5).sleep();
		ros::spinOnce();
	}
	objects_detected = false; // restore variable
	sub.shutdown();			  // free the subscriber, we don't need it anymore

	// REACH THE OPPOSITE SIDE OF THE TABLE to detect all the objects (setting 3 intermediate poses to avoid table)
	ServiceClient movement_client;

	// reach the 3 intermediate poses:
	movement_client.send_goal(8.8, -2.0, M_PI / 2); // bottom left
	while ((ros::ok()) && (!goal_reached))
	{
		ros::Duration(2.5).sleep();
		ros::spinOnce();
	}
	goal_reached = false; // restore variable

	movement_client.send_goal(8.8, -4.0, M_PI / 2); // top left
	while ((ros::ok()) && (!goal_reached))
	{
		ros::Duration(2.5).sleep();
		ros::spinOnce();
	}
	goal_reached = false; // restore variable

	movement_client.send_goal(7.8, -4.0, -M_PI / 2); // final pose in front of the table
	while ((ros::ok()) && (!goal_reached))
	{
		ros::Duration(2.5).sleep();
		ros::spinOnce();
	}
	goal_reached = false; // restore variable

	// DETECT REMAINING OBJECTS from this other point of view
	sub = nh.subscribe("tag_detections", 1, tagCallback);
	while ((ros::ok()) && (!objects_detected))
	{
		ros::Duration(2.5).sleep();
		ros::spinOnce();
	}
	sub.shutdown();

	// PUBLISH MESSAGE WITH INFO FOR NODEC: PICK AND PLACE
	// in the message we include coordinates for each object detected, coordinates are in map reference frame
	// it's not possible to send multidimensional arrays as messages in ros so, this is best solution
	ros::Publisher pub = nh.advertise<tiago_iaslab_simulation::pick_info>("message", 1);
	ROS_INFO("Creating the message with information for nodeC");
	tiago_iaslab_simulation::pick_info msg;

	msg.id_order.resize(3);
	for (int i = 0; i < 3; i++)
	{
		msg.id_order[i] = id_order[i];
	}
	msg.obj1_pos.resize(3);
	for (int i = 0; i < 3; i++)
	{
		msg.obj1_pos[i] = objects[0][i];
	}
	msg.obj2_pos.resize(3);
	for (int i = 0; i < 3; i++)
	{
		msg.obj2_pos[i] = objects[1][i];
	}
	msg.obj3_pos.resize(3);
	for (int i = 0; i < 3; i++)
	{
		msg.obj3_pos[i] = objects[2][i];
	}
	msg.obj4_pos.resize(3);
	for (int i = 0; i < 3; i++)
	{
		msg.obj4_pos[i] = objects[3][i];
	}
	msg.obj5_pos.resize(3);
	for (int i = 0; i < 3; i++)
	{
		msg.obj5_pos[i] = objects[4][i];
	}
	msg.obj6_pos.resize(3);
	for (int i = 0; i < 3; i++)
	{
		msg.obj6_pos[i] = objects[5][i];
	}
	msg.obj7_pos.resize(3);
	for (int i = 0; i < 3; i++)
	{
		msg.obj7_pos[i] = objects[6][i];
	}

	ROS_INFO("Message published from nodeB -> nodeC starts");

	// let's finally send the message, nodeC is waiting for it
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
