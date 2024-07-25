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
	      result_.obstacles.clear();//empty the vector before start
	      feedback_.status.clear();

	      feedback_.status = "The robot is ready";
	      as_.publishFeedback(feedback_);

	      if (as_.isPreemptRequested() || !ros::ok())
	      {
		ROS_INFO("%s: Preempted", action_name_.c_str());
		as_.setPreempted();
		success = false;
	      }

	      float x = goal->pose_b[0];
	      float y = goal->pose_b[1];
	      float z = goal->pose_b[2];
	      ROS_INFO("Recived goal pose_b: x=%f y=%f z=%f",x,y,z);
	      feedback_.status = "The server recived the goal: Pose_B";
	      as_.publishFeedback(feedback_);

	     
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	      //MOVE THE ROBOT TO POSE_B
	      // Create an action client for the move_base action server
	      MoveBaseClient ac("move_base", true);

	      // Wait for the action server to come up
	      while(!ac.waitForServer(ros::Duration(5.0))){
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
             quaternion.setRPY(0, 0, z);  // Roll, pitch, yaw (rotate about z-axis di 90 gradi: M_PI = 3,14)
             goal2.target_pose.pose.orientation.x = 0.0;  // Assuming no rotation about x-axis
             goal2.target_pose.pose.orientation.y = 0.0;  // Assuming no rotation about y-axis
             goal2.target_pose.pose.orientation.z = -quaternion.z();
             goal2.target_pose.pose.orientation.w = quaternion.w();

             ROS_INFO("Sending goal to move_base");
		  
	     // Send the goal to the move_base action server
	     ac.sendGoal(goal2);
	     feedback_.status = "The robot is mooving";
	     as_.publishFeedback(feedback_);
	     
	     // Wait for the result of the goal
	     ac.waitForResult();
	     
	     // Check if the goal was successful
	     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	         ROS_INFO("Hooray, the robot reached the goal");
		 feedback_.status = "The robot reached the goal: Pose_B";
		 as_.publishFeedback(feedback_);
	     }
	     else{
	         ROS_INFO("The base failed to move forward 1 meter for some reason");
		 feedback_.status = "Some problem has occured, the robot didn't reach the goal";
		 as_.publishFeedback(feedback_);
	     }
	      
	      
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	      
	      
	    // Use boost::bind to bind the member function to the current instance of the class, to modify non-static variables in static function
	    boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)> callbackFn =
	    boost::bind(&ServerAction::scanCallback, this, _1);
		
		
	    //Scan laser data
	    feedback_.status = "The robot starts the detection of obstacles";
	    as_.publishFeedback(feedback_);
	    // Subscribe to the /scan topic
	    ros::Subscriber scan_sub = nh_.subscribe("/scan", 1, callbackFn);
	    
	    // Spin to allow callback function to run
	    while (ros::ok()) {
	        ros::spinOnce(); // Allow callback function to run
		// Check if the callback has been executed
		if (callbackExecuted) {
		    break; // Break out of the loop after the first execution
		}
		r.sleep();
	    }


	    if (success)
	    {
	      ROS_INFO("%s: Succeeded", action_name_.c_str());
	      as_.setSucceeded(result_);
	      feedback_.status = "The robot has ended the detection";
	      as_.publishFeedback(feedback_);
	    }
	  }
	  


	  
	  //Scan laser data
	  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
	  {
	   
	      // Object detection parameters
	      const float detection_threshold = 0.1;    // Min difference between consecutive scans to be classified as potential cilindric object
	      const int min_consecutive_ranges = 10;    // Minimum consecutive ranges to classify as an object

	      bool is_start = false; //detected start of movable object (we want to detect the end of an object, its last scan)
	      int starting_object_scan = 0;
	    
	      for (size_t i = 21; i < scan_msg->ranges.size() - 22; ++i) {//scan the ranges array
	          float range = scan_msg->ranges[i];
		  float prev_range = scan_msg->ranges[i - 1];
		  float next_range = scan_msg->ranges[i + 1];

		  // Calculate percentage difference between consecutive scans
		  float diff_percentage = std::abs(range - prev_range) / prev_range;

		  // Check if the percentage difference is above the threshold
		  if ((diff_percentage > detection_threshold)&&(diff_percentage < 8.0)) {	//there is a big difference on 2 consecudive scans
		  
		      if(is_start == false){//here the object starts
		    	  is_start = true;
		    	  starting_object_scan = i;//to detect the length of the object

		      }
		      else{//here the object ends
		    	  int object_scans = i - starting_object_scan;
		    	  if((object_scans > min_consecutive_ranges)&&(object_scans < 100)){	//detect only our cilindric objects

		              //this is a circular object -> object detected
		    	      int object_center = round(i - (object_scans/2));//scan at center of cilinder
		              double angle = scan_msg->angle_min + object_center * scan_msg->angle_increment;
		              float x = prev_range * cos(angle);//polar to cartesan coordinates
		              float y = prev_range * sin(angle);
		              ROS_INFO("Object detected at x= %f, y= %f", x, y);
		            
		              // Add detected object to the result
			      result_.obstacles.push_back(x);
			      result_.obstacles.push_back(y);
			      feedback_.status = "The robot finded an obstacle";
			      as_.publishFeedback(feedback_);
		    	}
		    	is_start = false;
		    }
		}
	    }
	    callbackExecuted = true;
	    nh_.shutdown();
	  }
	 
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_action_server");
  ServerAction server("fibonacci");
  ros::spin();
  return 0;
}


