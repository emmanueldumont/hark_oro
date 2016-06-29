/**
  * \file harkListener.h
  * \brief This file contains headers and functions used to manages hark in oroclient
  * \author DUMONT Emmanuel
  * \date 04/2016
  */

#ifndef HARKLISTENER_H
#define HARKLISTENER_H


#include <stdint.h>

//#include "std_msgs/String.h"

typedef struct sourceValues
{
	int32_t timeStamp; //last time heard
	int id; // Id of the source
	float x;	// X position
	float y;	// Y postion
	float z;	// Z position
	float angle;	// Azimuth
}t_sourceValues;


/*enum enCommand
{
    CMD_ADD_INST =1,
    CMD_ADD_PROP,
    CMD_FIND,
    CMD_REMOVE,
    CMD_CLEAR,
    LAST_CMD
};*/


void process_Sources(t_sourceValues * srvVal, int size);
//void sayMyName();
void clearSource(int id);
void updateSourceData(int id, char data, int value);
void updateSourceAngle(int id, int angle);
void updateTimestamp(int id);
void addSource(t_sourceValues * soundSource);
//void oroChatterSender(std_msgs::String msg);


// Global database of sources
#define NB_SOURCE_VALUES_MAX 3

// Define the max euclidian distance
#define MAX_EUCLIDIAN_DISTANCE 50 // Centimeter
#define MAX_ANGLE_DISTANCE 10 // Degrees
#define MAX_TIME 3000 // Keep messages max 1 sec
//#define TIME_BEFORE_UPDATE 3000

/*#if MAX_TIME - TIME_BEFORE_UPDATE < 1
#error error harkListener.hpp : MAXTIME and TIME BEFORE UPDATE are not correct value -> timeStamp refresh is not possible
#endif*/


#endif
