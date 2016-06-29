/**
  * \file harkListener.cpp
  * \brief This file contains functions used to manages hark in oroclient
  * \author DUMONT Emmanuel
  * \date 04/2016
  */



#include <ros/ros.h>
#include <ros/package.h>

#include "harkListener.hpp"
#include "hark_msgs/HarkSource.h"
#include "hark_msgs/HarkSourceVal.h"

#include <string>
#include <vector>
#include <time.h>
#include <math.h>

#include <iostream>
#include <fstream>
#include <sstream>


#include <signal.h>

#include "std_msgs/String.h"
#include <stdexcept>


#include <orolib/orolib.hpp>

//using namespace std;



t_sourceValues database[NB_SOURCE_VALUES_MAX]; // Global variables of sources
int gNbSourceValues; // Current number of sources values registered

// Chatter able to communicate with oroclient
//ros::Publisher gOroChatter_pub;

// Name of the current node in the ontology
char * gName;
bool gToBeFree; // Boolean which indicates if gName has to be freed

std::string room;


// Create a function which manages a clean "CTRL+C" command -> sigint command
void sigint_handler(int dummy)
{
    ROS_INFO("- hark_oro is shutting down...");
    
    // Free gName
    if(gToBeFree == true)
    {
      gToBeFree = false;
      free(gName);
    }
        
    ROS_INFO("\n\n... Bye bye !\n   -Manu");
    exit(EXIT_SUCCESS); // Shut down the program
}



void chatterCallback(const hark_msgs::HarkSource::ConstPtr& msg)
{
	// Check if msg is not empty
	if( msg != NULL)
	{	  
	  int tempSize = msg->src.size();
	  if (tempSize > 0)
	  {
		  t_sourceValues * srvVal = NULL;		  
		
		  srvVal = (t_sourceValues*) malloc(msg->src.size() * sizeof(t_sourceValues));
		  
		  if(srvVal != NULL)
		  {
		
			  // Retrieve Sources Position and put it in entry
			  //     Azimuth stores the angle
			  stringstream entry;
			  entry << "Received data:" << "(" << msg->count << "|" << tempSize;
			  for (int i=0; i < tempSize ; ++i)
			  {
				  // Get data from the sources stored in msg
				  //const hark_msgs::HarkSourceVal &data = msg->src[i];
				  //entry << " | " << "src" << data.id  << ",x " << data.x << ",y " << data.y << ",z " << data.z << ",az " << data.azimuth;
				  entry << " | " << "src" << msg->src[i].id  << ",x " << msg->src[i].x << ",y " << msg->src[i].y << ",z " << msg->src[i].z << ",az " << msg->src[i].azimuth;
				
				  srvVal[i].timeStamp = msg->count;
				  srvVal[i].id = msg->src[i].id;
				  srvVal[i].x = msg->src[i].x;
				  srvVal[i].y = msg->src[i].y;
				  srvVal[i].z = msg->src[i].z;
				  srvVal[i].angle = msg->src[i].azimuth;
				
				  //entry << " | " << "src" << srvVal[i].id  << ",x " << srvVal[i].x << ",y " << srvVal[i].y << ",z " << srvVal[i].z << ",az " << srvVal[i].angle;
			  }
			  entry << ")";
		
			  // Display the constructed String
			  std::string s = entry.str();
		
			  //Treatment done on the received packet
			  process_Sources(srvVal, msg->src.size());
			  
			  free(srvVal);
		  }
    }
  }
  else
  {
  	ROS_INFO("- Warning: harkCallback, message empty");
  }
  
}




int main(int argc, char **argv)
{
	// Initialize global variables
	gNbSourceValues = 0;
	gToBeFree = false;
	
	for(int i =0; i <NB_SOURCE_VALUES_MAX ;i++)
	{
		database[i].timeStamp= 0;
		database[i].id = 0;
		database[i].x= 0;
		database[i].y= 0;
		database[i].z= 0;
		database[i].angle= 0;
	}
	
  if(argc > 1)
  {
    ros::init(argc, argv, "hark_listener1");
  }
  else
  {
    ros::init(argc, argv, "hark_listener");
  }

  ros::NodeHandle nHark;  // Read from hark topic
  ros::NodeHandle nOroCl;  // Communicate wit oroclient
  
  // Override the signal interrupt from ROS
  signal(SIGINT, sigint_handler);
  
  // If a name is passed as an argument
  ros::Subscriber sub;// = nHark.subscribe("HarkSource", 1000000, chatterCallback);

  if(argc > 1)
  {
    gName = argv[argc-1];
    gToBeFree = false;
    sub = nHark.subscribe("HarkSource", 1000000, chatterCallback);
  }
  else
  {
    gName = (char *) malloc ( 7 * sizeof(char));
    memset(gName, 0, 7);
    snprintf(gName, 7 ,"mK");
     
    gToBeFree = true;
     sub = nHark.subscribe("HarkSource2", 1000000, chatterCallback);
  }


  
  init_oroChatter();
  
  usleep(500000); // necessary to be able to send data
  
  std::string sensor = "Microphone";
  if(argc > 1)
  {
    room = "LivingRoom";
  }
  else
  {
    room = "Kitchen";
  }
  
  // say my name
  sayMyName(gName, sensor);
  sayMyRoom(gName, room);
  
  ros::spin();

  return 0;
}


void process_Sources(t_sourceValues * srvVal, int size)
{
  for(int cpt =0; cpt <gNbSourceValues; cpt++ )
  {
        printf("database '%d' timestamp '%d'\n",cpt,database[cpt].timeStamp);
  }
  printf("-------------\n\n\n");

	// If there are sources values coming from Hark
  if(size > 0)
  {
    
    // 1- It is a new source
    //    Then add it
    // 2- It is not a new source
    //    update what should be
    
    for(int cpt =0; cpt < size; cpt ++)
    {
      bool next = false; // Boolean which indicates if we can skip next parts or keep going in treatment
      //-------------------------------
      // 1- Check if it is an old source
      //-------------------------------
      int cptCheck = 0; // Counter
      do
		  {
		    if(srvVal[cpt].id == database[cptCheck].id)
		    {
		      // Check if something changed in each parameter, if so, change them in ontology
		      database[cptCheck].timeStamp= srvVal[cpt].timeStamp; // Automatically update timestamp
		      if(srvVal[cpt].x != database[cptCheck].x) { updateSourceData(srvVal[cpt].id, 'x', srvVal[cpt].x); database[cptCheck].x= srvVal[cpt].x; }
		      if(srvVal[cpt].y != database[cptCheck].y) { updateSourceData(srvVal[cpt].id, 'y', srvVal[cpt].y); database[cptCheck].y= srvVal[cpt].y; }
		      if(srvVal[cpt].z != database[cptCheck].z) { updateSourceData(srvVal[cpt].id, 'z', srvVal[cpt].z); database[cptCheck].z= srvVal[cpt].z; }
		      if(srvVal[cpt].angle != database[cptCheck].angle) { updateSourceAngle(srvVal[cpt].id, srvVal[cpt].angle); database[cptCheck].angle= srvVal[cpt].angle; }
		      
		      next = true;
		      break;
		    }
		    
		    cptCheck += 1;
		    
		  }while( cptCheck < gNbSourceValues );
		  
		  // If modifcation occured, then work on next source received
		  if(next == true) continue;
		  
		  //else...
		  
		  //--------------------------------------
      // 2- Check if it is a 'false new source
      //--------------------------------------
      
      // If two different sources are close, it meeans the are identical.
      // Check 1st nearest neighboor
      int cptKNN = 0;
      double distance = 0;
      do
			{
			  distance = 0;
			  // If it is not an previously initialized source
			  if(database[cptKNN].timeStamp != 0 )
			  {
			    // Determine the angle difference:
			    distance = abs(database[cptKNN].angle - srvVal[cpt].angle);
			    
			    // If the angle difference is small then it might be the same sources
			    if( distance < MAX_ANGLE_DISTANCE )
			    {
			      // Determine the euclidian distance : sqrt( ( x1-x2 )^2 + ( y1-y2 )^2 + ( z1-z2 )^2 )
			      distance = sqrt( pow(database[cptKNN].x - srvVal[cpt].x, 2) + pow(database[cptKNN].y - srvVal[cpt].y, 2) + pow(database[cptKNN].z - srvVal[cpt].z, 2) );
			      
			      if(distance < MAX_EUCLIDIAN_DISTANCE)
			      {
			        // Same point: Update the timestamp
			        database[cptKNN].timeStamp = srvVal[cpt].timeStamp;
			        
			        // BREAKPOINT: if a corresponding point is found, analyze next source, if there is echo, filters will remove it later
			        next = true; // Next now indicates if a similar point has been found
			        break;
			      }
			    }
			    // If it was not close enough to be considered as the same source, then loop to next stored souce
				  cptKNN +=1;
				  
				}
				else
				{
				  // If timestamp is equal to 0 it means we are at the end so, do not loop again
				  break;
				}
				
			}while( cptKNN < gNbSourceValues );
			
			// If next is true, it means it had a nearest neighboor, so do not add it
			if(next == true) continue;
			
			// Otherwise it means it is a real new source...
			
			//----------------------
			// 3- Add the new source
			//----------------------
		  if( (gNbSourceValues+1) < NB_SOURCE_VALUES_MAX)
			{
				database[gNbSourceValues].timeStamp = srvVal[cpt].timeStamp;
				database[gNbSourceValues].id= srvVal[cpt].id;
				database[gNbSourceValues].x= srvVal[cpt].x;
				database[gNbSourceValues].y= srvVal[cpt].y;
				database[gNbSourceValues].z= srvVal[cpt].z;
				database[gNbSourceValues].angle= srvVal[cpt].angle;
				
				ROS_INFO("ADD s%s%d",gName,database[gNbSourceValues].id);
        addSource(&database[gNbSourceValues]);
        
				gNbSourceValues += 1; // Add one new sources
			}
    }
    
    //-------------------
    // Remove old sources
    //-------------------
    
    // First resort the source array from youngest to oldest
    int table[2][gNbSourceValues];  // Table which will facilitate future work
    
    // Clean table
    fill_n(table[0], gNbSourceValues, 0);
    fill_n(table[1], gNbSourceValues, 0);
    
    // Create a buffer array of sources
    t_sourceValues buffSrc[gNbSourceValues];
    
    // Init the buffer with current value in the global array
    for(int cpt = 0; cpt < gNbSourceValues; cpt++)
    {
      buffSrc[cpt].timeStamp= database[cpt].timeStamp;
      buffSrc[cpt].id = database[cpt].id;
      buffSrc[cpt].x= database[cpt].x;
      buffSrc[cpt].y= database[cpt].y;
      buffSrc[cpt].z= database[cpt].z;
      buffSrc[cpt].angle= database[cpt].angle;
    }
    
    //-> Sort sources by timestamp: From recent to older
    for(int cptSort=0; cptSort < gNbSourceValues; cptSort++)
    {
      // Get the timestamp and store it
      table[0][cptSort] = database[cptSort].timeStamp;
    }
    
    int smallest = 0;
    int cptSmall = 0;
    // Find greatest timestamp, put it on left and loop
    for(int cptAll=0; cptAll < gNbSourceValues; cptAll++)
    {
      // Initialize value of the youngest data
      smallest = cptAll;
      
      // Check if the youngest chosen is the final youngest
      // Youngest means highest value to the current timestamp
      for(cptSmall = cptAll+1; cptSmall < gNbSourceValues; cptSmall++)
      {
        if( table[0][smallest] < table[0][cptSmall] ) {
          smallest = cptSmall;}
      }
      table[1][cptAll] = smallest;
    }
    
    int nbSrcRmv = 0; // Number of sources removed
    int currentTime = srvVal[0].timeStamp; // Current timestamp
    
    // Sort the global table
    for(int cpt = 0; cpt < gNbSourceValues; cpt++)
    {
      // If timestamp is not too old, copy data in global table
      //ROS_INFO("curr '%d' CPTTime '%d'",currentTime , buffSrc[ table[1][cpt] ].timeStamp);
      
      if( (currentTime - buffSrc[ table[1][cpt] ].timeStamp)  < MAX_TIME )
      {
        database[cpt].timeStamp = buffSrc[ table[1][cpt] ].timeStamp;
        database[cpt].id = buffSrc[ table[1][cpt] ].id;
        database[cpt].x = buffSrc[ table[1][cpt] ].x;
        database[cpt].y = buffSrc[ table[1][cpt] ].y;
        database[cpt].z = buffSrc[ table[1][cpt] ].z;
        database[cpt].angle = buffSrc[ table[1][cpt] ].angle;
      }
      else // else remove them in the ontology and store a zero data
      {
      
        ROS_INFO("Clear too old s%s%d",gName,database[cpt].id);
        clearSource(buffSrc[ table[1][cpt] ].id);
        
        database[cpt].timeStamp = 0;
        database[cpt].id = 0;
        database[cpt].x = 0;
        database[cpt].y = 0;
        database[cpt].z = 0;
        database[cpt].angle = 0;
        
        nbSrcRmv +=1;
      }
    }
  
    // Update number of source values in the global array
    gNbSourceValues = gNbSourceValues - nbSrcRmv;
    
	}
	else
	{
		ROS_INFO("- Warning: Not enough sources from Hark to process");
	}
}



// Explain who am i
/*void sayMyName()
{
  ros::Rate loop_rate(10); // Communicate slow rate

  std::stringstream ss;
  char enumCmd = (char)CMD_ADD_INST;
  
  ss << "BigBrother#"<< enumCmd <<"#"<<gName<<"#Microphone";
  
  std_msgs::String msg;
  msg.data = ss.str();
  oroChatterSender(msg);
}*/

// Clear a source in the ontology
void clearSource(int id)
{
  std::stringstream ss;
  //char enumCmd = (char)CMD_CLEAR;
  char enumCmd = (char)CMD_REMOVE;
  
  //ROS_INFO("Clear source : s%d",id);
  
  ss.str("");
  ss << "BigBrother#"<< enumCmd <<"#s" << gName << id ;//<<"#?x#?y";
  
  std_msgs::String msg;
  msg.data = ss.str();
  oroChatterSender(msg);
}

// Update a specific data of a source
void updateSourceData(int id, char data, int value)
{
  std::stringstream ss;
  char enumCmd = 0;
  
  //ROS_INFO("Update %c in source : s%d", data,id);
  
  // Clear timestamp otherwise update won't be done correctly
  ss.str("");
  enumCmd = (char)CMD_CLEAR;
  ss << "BigBrother#"<<enumCmd<<"#s" << gName << id <<"#"<< data <<"#?y";
  std_msgs::String msg;
  msg.data = ss.str();
  oroChatterSender(msg);
  
  // "Update" in oro server seems to not work correctly, re-add instead
  ss.str("");
  enumCmd = (char)CMD_ADD_PROP;
  ss << "BigBrother#" << enumCmd <<"#s" << gName << id << "#"<< data <<"#"<< value;
  
  msg.data = ss.str();
  oroChatterSender(msg);
}

void updateSourceAngle(int id, int angle)
{
  std::stringstream ss;
  char enumCmd = 0;
  
  //ROS_INFO("Update source's angle : s%d",id);
  
  // Clear timestamp otherwise update won't be done correctly
  ss.str("");
  enumCmd = (char)CMD_CLEAR;
  ss << "BigBrother#"<<enumCmd<<"#s" << gName << id <<"#angle#?y";
  std_msgs::String msg;
  msg.data = ss.str();
  oroChatterSender(msg);
  
  // "Update" in oro server seems to not work correctly, re-add instead
  ss.str("");
  enumCmd = (char)CMD_ADD_PROP;
  ss << "BigBrother#" << enumCmd <<"#s" << gName << id << "#angle" <<"#"<< angle;
  
  msg.data = ss.str();
  oroChatterSender(msg);
}

// Update a timestamp in a source in the ontology
void updateTimestamp(int id)
{
  std::stringstream ss;
  char enumCmd = 0;
  
  //ROS_INFO("Update source : s%d",id);
  
  // Gettime
  time_t rawtime;
  struct tm * timeinfo;    
  time ( &rawtime );
  
  // Clear timestamp otherwise update won't be done correctly
  ss.str("");
  enumCmd = (char)CMD_CLEAR;
  ss << "BigBrother#"<<enumCmd<<"#s" << gName << id <<"#timestamp#?y";
  std_msgs::String msg;
  msg.data = ss.str();
  oroChatterSender(msg);
  
  // "Update" in oro server seems to not work correctly, re-add instead
  timeinfo = localtime ( &rawtime );
  ss.str("");
  enumCmd = (char)CMD_ADD_PROP;
  ss << "BigBrother#" << enumCmd <<"#s" << gName << id << "#timestamp" <<"#"<< timeinfo->tm_hour <<"."<< timeinfo->tm_min;
  
  msg.data = ss.str();
  oroChatterSender(msg);
}

// Add a new source in the ontology
void addSource(t_sourceValues * soundSource)
{
  // Gettime
  time_t rawtime;
  struct tm * timeinfo;    
  time ( &rawtime );
  std::stringstream ss;
  char enumCmd = 0;
  
  // Construct string
  for(int cpt=0; cpt < 7; cpt++)
  {
    // Construct string to send
    switch(cpt)
    {
      // 0- Add the new sound
      case 0: ss.str("");
              enumCmd = (char)CMD_ADD_INST;
              ss << "BigBrother#"<<enumCmd<<"#s" << gName << soundSource->id <<"#Human";
      break;
      
      // 1- Add that we can hear it
      case 1:  ss.str("");
	            enumCmd = (char)CMD_ADD_PROP;
				      ss << "BigBrother#" << enumCmd <<"#"<< gName << "#canHear" <<"#s" << gName << soundSource->id;
      break;
      
      // 2- Add the sound's timestamp
      case 2:  // Gettime
              /*timeinfo = localtime ( &rawtime );
              ss.str("");
              enumCmd = (char)CMD_ADD_PROP;
			        ss << "BigBrother#" << enumCmd <<"#s" << gName << soundSource->id << "#timestamp" <<"#"<< timeinfo->tm_hour <<"."<< timeinfo->tm_min;*/
      break;
      
      // 3- Add the sound's X-position
      case 3: ss.str("");
              ss << "BigBrother#"<<enumCmd<<"#s" << gName << soundSource->id <<"#x#"<< soundSource->x;
      break;
      
      // 3- Add the sound's Y-position
      case 4: ss.str("");
              ss << "BigBrother#"<<enumCmd<<"#s" << gName << soundSource->id <<"#y#"<< soundSource->y;
      break;
      
      // 3- Add the sound's Z-position
      case 5: ss.str("");
              ss << "BigBrother#"<<enumCmd<<"#s" << gName << soundSource->id <<"#z#"<< soundSource->z;
      break;
      
      // 3- Add the sound's Angle-position
      case 6: ss.str("");
              ss << "BigBrother#"<<enumCmd<<"#s" << gName << soundSource->id <<"#angle#"<< soundSource->angle;
      break;
      
      default:
      break;
    }
    
    //ROS_INFO("Add source : s%d",soundSource->id);
    
    // Send it
    std_msgs::String msg;
    msg.data = ss.str();
    oroChatterSender(msg);
    
  }
}

/*
void oroChatterSender(std_msgs::String msg)
{  
  //ROS_INFO("%s", msg.data.c_str());
  
  gOroChatter_pub.publish(msg);
  ros::spinOnce();
  //loop_rate.sleep();
}*/
