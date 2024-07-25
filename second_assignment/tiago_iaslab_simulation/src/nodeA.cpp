// NODE THAT REACH THE POSE IN FRONT OF THE TABLE
#include <ros/ros.h>
#include <thread>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tiago_iaslab_simulation/ObstacleDetectionAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "tiago_iaslab_simulation/start_nodeB.h"

typedef actionlib::SimpleActionClient<tiago_iaslab_simulation::ObstacleDetectionAction> Client;

bool goal_reached = false; // became true when the goal is reached
bool head_moved = false;   // became true when the head is moved

// class that sends the goal to the action server
// the goal is the pose in front of the table
// the action server is the one that detects the cilindric objects
// the goal is to reach the table and detect the cilindric objects
class ServiceClient
{
public:
	// constructor that waits for the action server to start
	ServiceClient() : ac("fibonacci", true)
	{
		ROS_INFO("Waiting for action server to start.");
		ac.waitForServer();
		ROS_INFO("Action server started, sending pose goal.");
	}
	// send the goal to the action server
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

	// callback functions
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

// class that moves the head down to have a better view of the table by setting its 2 joints
class HeadMover
{
public:
	HeadMover() : nh("~")
	{
		head_cmd_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1);
	}

	// move the head down 
	void lowerHead()
	{

		trajectory_msgs::JointTrajectory jt;
		jt.joint_names = {"head_1_joint", "head_2_joint"};

		trajectory_msgs::JointTrajectoryPoint jtp;
		jtp.positions = {0.0, -0.65};
		jtp.time_from_start = ros::Duration(2.0);
		jt.points.push_back(jtp);

		head_cmd_pub.publish(jt);

		head_moved = true;
	}

private:
	ros::NodeHandle nh;
	ros::Publisher head_cmd_pub;
};

////////////////////////////////////////////////////////////////////////////////////////////
// MAIN
int main(int argc, char **argv)
{
	ros::init(argc, argv, "move");
	ros::NodeHandle nh;

	// REACH THE TABLE
	// start asking for the goal and moving to the goal pose:
	ServiceClient movement_client;

	ROS_INFO("Let's reach the table");
	movement_client.send_goal(8.0, 0.0, M_PI / 2); // intermediate pose to avoid cilindric obstacle

	while ((ros::ok()) && (!goal_reached))
	{
		// wait for the goal to be reached
		ros::Duration(2.5).sleep();
		ros::spinOnce();
	}
	goal_reached = false; // restore initial value

	movement_client.send_goal(7.8, -2.0, M_PI / 2); // final pose
	while ((ros::ok()) && (!goal_reached))
	{
		// wait for the goal to be reached
		ros::Duration(2.5).sleep();
		ros::spinOnce();
	}

	// setting torso as tall as possible by setting it's joint
	ROS_INFO("Setting the torso as tall as possible to have a better view");

	// MAKE THE ROBOT AS HIGH AS POSSIBLE TO SEE FROM A HIGHER POINT OF VIEW
	ros::AsyncSpinner spinner(1);
	spinner.start();

	static const std::string PLANNING_GROUP = "arm_torso";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	const robot_state::JointModelGroup *joint_model_group =
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	joint_group_positions[0] = 0.34;
	move_group.setJointValueTarget(joint_group_positions);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

	// Execute the planned trajectory
	moveit::planning_interface::MoveItErrorCode execution_result = move_group.execute(my_plan);
	ROS_INFO_NAMED("tutorial", "Execution result: %s", execution_result == moveit::planning_interface::MoveItErrorCode::SUCCESS ? "SUCCESS" : "FAILED");

	// MOVE HEAD DOWN to have a better view of the table
	HeadMover head_mover;

	// send message to nodeB to start the objects detection
	ros::Publisher pub = nh.advertise<tiago_iaslab_simulation::start_nodeB>("start_nodeB", 1);

	tiago_iaslab_simulation::start_nodeB msg;

	msg.status = 1;

	ROS_INFO("Moving the head down to point in the direction of the table and start the detection");
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

		// move head down
		head_mover.lowerHead();
		ros::Duration(2.0).sleep();
		ros::spinOnce();
	}

	return 0;
}
