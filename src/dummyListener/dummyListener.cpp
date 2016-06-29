/**
  * \file dummyListener.cpp
  * \brief This file contains dummy functions used to listen Hark_Ros
  * \author DUMONT Emmanuel
  * \date 02/2016
  */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hark_msgs/HarkSource.h"
#include "hark_msgs/HarkSourceVal.h"

using namespace std;



void chatterCallback(const hark_msgs::HarkSource::ConstPtr& msg)
{
	// Check if msg is not empty
	if( (msg != NULL) && (msg->src.size() > 0) )//&& (msg->src != NULL) && (msg->src.size() > 0))
	{
		// Retrieve Sources Position and put it in entry
		//     Azimuth stores the angle
		stringstream entry;
		entry << "Received data:" << "(" << msg->count ;
		for (int i=0; i<msg->src.size() ; ++i)
		{
			// Get data from the sources stored in msg
			const hark_msgs::HarkSourceVal &data = msg->src[i];
			entry << " | " << "src" << data.id  << ",x " << data.x << ",y " << data.y << ",z " << data.z << ",az " << data.azimuth;
		}
		entry << ")";
	
		// Display the constructed String
		std::string s = entry.str();
		ROS_INFO("%s", s.c_str());
  }
  else
  {
  	ROS_INFO("- Warning: harkCallback, message empty");
  }
}




int main(int argc, char **argv)
{
	
  ros::init(argc, argv, "hark_dummyListener");

  ros::NodeHandle nHark;  // Read from hark topic
 
  ros::Subscriber sub = nHark.subscribe("HarkSource", 1000000, chatterCallback);  
  
  ros::spin();

  return 0;
}
