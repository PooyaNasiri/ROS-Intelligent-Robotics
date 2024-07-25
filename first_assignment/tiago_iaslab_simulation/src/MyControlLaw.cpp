#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class CorridorNavigator {
public:
    CorridorNavigator() : nh_("~") {
        // Create a publisher for sending velocity commands
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);

        // Create a subscriber for laser scan data
        scan_sub_ = nh_.subscribe("/scan", 1, &CorridorNavigator::laserCallback, this);        
    }
    
    bool corridor = true;//became false when corridor ends to stop custom control law


    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {

        float min_distance = *std::min_element(scan_msg->ranges.begin()+20, scan_msg->ranges.end()-20); //minimum distance with objects

        if(min_distance > 1.0) {//end of corridor: we're in the big room
        	ROS_INFO("reached the end of corridor");
        	corridor = false;
        }
        else if (min_distance < 0.1) {//too near a wall, we must stop  
            ROS_INFO("Near a wall -> stop");
            sendVelocityCommand(0.0, 0.0);  // Stop
        } else {
            if(scan_msg->ranges[25] > scan_msg->ranges[scan_msg->ranges.size()-25]){//first 20 and last 20 scans have noise
            	sendVelocityCommand(0.5, -0.05);//go back to center of corridor
            }else{
            	sendVelocityCommand(0.5, 0.05);
            }
        }
    }


    void sendVelocityCommand(double linear, double angular) {
        // create a Twist message and publish it
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = linear;
        twist_msg.angular.z = angular;
        cmd_vel_pub_.publish(twist_msg);
    }


    void run() {
	    ros::Rate r(10);//frequency 10 hz
	    while (ros::ok() && corridor) {//while we're in corridor
		ros::spinOnce(); // Allow callback function to run
		r.sleep();
	    }
	    //end of corridor
	    ROS_INFO("Ending our own motion control law, use move_base stack now");
	    std::string command = "rosrun tiago_iaslab_simulation MyActionClient";
    	    int result = std::system(command.c_str());	//start ServiceClient node
    	    if (result == 0) {
		ros::shutdown();  // This will stop the ROS node
    	    }	
	    
	}

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber scan_sub_;
};


////////////////////////////////////////////////////////////////////////////////////////





int main (int argc, char **argv)
{
  ros::init(argc, argv, "my_control_law");
  
  //EXTRA POINT:motion control law:
  CorridorNavigator navigator;
  ROS_INFO("Starting our own motion control law");
  navigator.run();
  
  //IMPORTANT: wait 10/15 seconds for the enviroment configuration before running this file

   ROS_INFO("Ending our own motion control law, use move_base stack now");
  
  
  return 0;
}

