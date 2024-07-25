#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tiago_iaslab_simulation/ObstacleDetectionAction.h>

typedef actionlib::SimpleActionClient<tiago_iaslab_simulation::ObstacleDetectionAction> Client;

class ServiceClient
{
	public:
	  ServiceClient() : ac("fibonacci", true)
	  {
	    ROS_INFO("Waiting for action server to start.");
	    ac.waitForServer();
	    ROS_INFO("Action server started, sending goal.");
	  }


	  void send_goal(float x, float y, float z)//reaches pose_b and detects cilindric objects
	  {

	    tiago_iaslab_simulation::ObstacleDetectionGoal goal;
	    float goal_pose [3] = {x,y,z};	//vector of 3 float for x position, y position and orientation

	    goal.pose_b.push_back(x);
	    goal.pose_b.push_back(y);
	    goal.pose_b.push_back(z);

	    ac.sendGoal(goal,
		        boost::bind(&ServiceClient::doneCb, this, _1, _2),
		        boost::bind(&ServiceClient::activeCb, this),
		        boost::bind(&ServiceClient::feedbackCb, this, _1)); 
	  }
	  

	  void doneCb(const actionlib::SimpleClientGoalState& state,
		      const tiago_iaslab_simulation::ObstacleDetectionResultConstPtr& result)
	  {
	    ROS_INFO("Finished in state [%s]", state.toString().c_str());
	    // Print each value in the 'obstacles' array
	    for (size_t i = 0; i < result->obstacles.size(); ++i)
	    {
	        ROS_INFO("Obstacle %zu: x = %f  y = %f", i/2, result->obstacles[i], result->obstacles[i+1]);
	        i++;      
	    }
	    ros::shutdown();
	  }
	  

	void activeCb()
	{
	  ROS_INFO("Goal just went active");
	}


	void feedbackCb(const tiago_iaslab_simulation::ObstacleDetectionFeedbackConstPtr& feedback)
	{
	    ROS_INFO("%s", feedback->status.c_str());//print feedbacks
	}


	private:
	  Client ac;
};



////////////////////////////////////////////////////////////////////////////////////////




int main (int argc, char **argv)
{
  ros::init(argc, argv, "my_action_client");
  
  
  
  
  //start asking for the goal and moving to pose_b:
  ServiceClient client;
  
  // Prompt the user to input x, y, and z values for pose_B
  float x, y, z;
  std::cout << "Enter x coordinate for pose_B (suggested: 11.0): ";
  std::cin >> x;
  std::cout << "Enter y coordinate for pose_B (suggested: 0.0): ";
  std::cin >> y;
  std::cout << "Enter z orientation for pose_B (in radians, suggested: 1.04(pi/3)): ";
  std::cin >> z;
  
  client.send_goal(x,y,z);
  ros::spin();
  
  
  return 0;
}

