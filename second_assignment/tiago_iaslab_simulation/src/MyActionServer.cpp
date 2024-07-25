#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tiago_iaslab_simulation/ObstacleDetectionAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/LaserScan.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ServerAction
{
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<tiago_iaslab_simulation::ObstacleDetectionAction> as_;
	std::string action_name_;
	tiago_iaslab_simulation::ObstacleDetectionFeedback feedback_;
	tiago_iaslab_simulation::ObstacleDetectionResult result_;
	bool callbackExecuted;

public:
	ServerAction(std::string name) : as_(nh_, name, boost::bind(&ServerAction::executeCB, this, _1), false), action_name_(name), callbackExecuted(false)
	{
		as_.start();
	}

	~ServerAction(void)
	{
	}

	void executeCB(const tiago_iaslab_simulation::ObstacleDetectionGoalConstPtr &goal)
	{
		ros::Rate r(1);
		bool success = true;
		result_.obstacles.clear(); // empty the vector before start
		feedback_.status.clear();

		// feedback_.status = "The robot is ready";
		// as_.publishFeedback(feedback_);

		if (as_.isPreemptRequested() || !ros::ok())
		{
			ROS_INFO("%s: Preempted", action_name_.c_str());
			as_.setPreempted();
			success = false;
		}

		float x = goal->pose_b[0];
		float y = goal->pose_b[1];
		float z = goal->pose_b[2];
		ROS_INFO("Recived goal pose: x=%f y=%f z=%f", x, y, z);
		// feedback_.status = "The server recived the goal pose";
		// as_.publishFeedback(feedback_);

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// MOVE THE ROBOT TO POSE_B
		//  Create an action client for the move_base action server
		MoveBaseClient ac("move_base", true);

		// Wait for the action server to come up
		while (!ac.waitForServer(ros::Duration(5.0)))
		{
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		// Create a goal for the robot to move to goal position
		move_base_msgs::MoveBaseGoal goal2;
		goal2.target_pose.header.frame_id = "map";
		goal2.target_pose.header.stamp = ros::Time::now();
		goal2.target_pose.pose.position.x = x;
		goal2.target_pose.pose.position.y = y;

		// Calculate the quaternion components for a 90-degree rotation about the z-axis
		tf2::Quaternion quaternion;
		quaternion.setRPY(0, 0, z);					// Roll, pitch, yaw (rotate about z-axis di 90 gradi: M_PI = 3,14)
		goal2.target_pose.pose.orientation.x = 0.0; // Assuming no rotation about x-axis
		goal2.target_pose.pose.orientation.y = 0.0; // Assuming no rotation about y-axis
		goal2.target_pose.pose.orientation.z = -quaternion.z();
		goal2.target_pose.pose.orientation.w = quaternion.w();

		ROS_INFO("Sending goal");

		// Send the goal to the move_base action server
		ac.sendGoal(goal2);
		feedback_.status = "The robot is mooving";
		as_.publishFeedback(feedback_);

		// Wait for the result of the goal
		ac.waitForResult();

		// Check if the goal was successful
		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Hooray, the robot reached the goal");
			// feedback_.status = "The robot reached the goal pose";
			// as_.publishFeedback(feedback_);
		}
		else
		{
			feedback_.status = "Some problem has occured, the robot didn't reach the goal";
			as_.publishFeedback(feedback_);
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "action_server");
	ServerAction server("fibonacci");
	ros::spin();
	return 0;
}
