#ifndef TIMESYNCMSG_H
#define TIMESYNCMSG_H
 
 enum {
	AM_TIMEMSG = 2,
 };

 typedef nx_struct timemsg{
	nx_uint16_t remNodeID;
	nx_uint32_t remTime;
	nx_uint16_t locNodeID;
	nx_uint32_t locTime;
 }timemsg_t;
 
 #endif


