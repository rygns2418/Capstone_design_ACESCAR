#include <iostream>
#include <vector>
#include <algorithm>
#include "zed_depth.h"
#include "lds_driver.h"
#include "control.h"
#include "ros/ros.h"
#include "lds_car/MsgControl.h"

#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef __CV_STD_NAMESPACE__
#define __CV_STD_NAMESPACE__
using namespace cv;
using namespace std;
#endif

bool corner_compensation=false;

int main(int argc,char** argv)
{

    sl::Camera zed;
    sl::InitParameters init_params;
    // Set runtime parameters after opening the camera
    sl::RuntimeParameters runtime_parameters;
    zed_init(init_params, zed, runtime_parameters);

	//ROS initialize
    ros::init(argc,argv,"TX2");
    ros::NodeHandle nh;
    ros::Publisher talker = nh.advertise<lds_car::MsgControl>("ros_msg",100);
    lds_car::MsgControl msg;

	//lidar initialize
	std::string port;
	int baud_rate;
	uint16_t rpms;
	port = "/dev/ttyUSB0";
	baud_rate = 230400;
	boost::asio::io_service io;
	lds::LFCDLaser laser(port, baud_rate, io);

	//shared memory initialize
	int shmid;
    int skey = 5678;
    
    int *shared_memory;
    
    shmid = shmget((key_t)skey, sizeof(int), 0666);
    if(shmid == -1)
    {
        perror("shmget failed\n");
        exit(0);
    }
    
    shared_memory = (int *)shmat(shmid, (void *)0, 0);
    if(!shared_memory)
    {
        perror("shmat failed : ");
        exit(0);
    }
    //printf("shm id : %d\n", shmid);
	int* num = (int *)shared_memory;

	// declear local variables
    sl::Mat image_zed;
    cv::Mat image_origin;

    vector<double> pnt(5, -1); 
    vector<double> fntPnt(5, -1);
    vector<double> lr(2,-1);
    
    int steeringInfo;
    int	isGoStraight;
    int isCorner;
     
    while(true)
    {
		try
		{
			laser.poll(pnt, fntPnt, lr);
			//made by kim kyo's main 
			
			isGoStraight = checkGoStraight(pnt, lr);
			if(corner_compensation && (steeringInfo > 86) && (steeringInfo < 150) && (pnt[2] == 3000))
			{
				if((pnt[3] < 2) && (pnt[4] < 1))
				{
					steeringInfo = 87;
					corner_compensation = false;
					
				}
				else
					steeringInfo =105;
			}
			else if(isGoStraight == 2)
			{
				steeringInfo = 70;
			}
			else if(isGoStraight == 0)//keep going straight
			{
				if(abs(pnt[4] - pnt[0]) > abs(pnt[3] - pnt[1]))
					steeringInfo = secondGetSteering(pnt[1], pnt[3]);
				else
					steeringInfo = firstGetSteering(pnt[0], pnt[4]);
			}   
			else if(isGoStraight == 1)// not going straight and c_point is near by wall
			{
				isCorner = checkCorner(fntPnt); // check is corner
				if(isCorner ==3) steeringInfo = thirdGetSteering(fntPnt[0], fntPnt[4]);
				else if(isCorner == 2) steeringInfo = thirdGetSteering(fntPnt[0], fntPnt[4]);
				else if(isCorner == 4) 
				{
					steeringInfo = 29;
					
					corner_compensation = true;
					
				}
			}
			else
				steeringInfo = 87;
			
			//publish steering info
            msg.data = steeringInfo;
            talker.publish(msg);
            if(num[0] != -1 && num[1] != -1)
            {
				getCVImage(zed, runtime_parameters, image_zed, image_origin);
				if(!image_origin.empty())
				{
					double d;
					getDepth(zed,num[0],num[1], d);
					if(d < 2.5)
					{
						msg.data = 9999;
						talker.publish(msg);
						#ifdef __DEBUG__
						printf("stop\n");
						#endif
					}
					#ifdef __DEBUG__
					printf("stop sign y : %d, x = %d\n", num[0], num[1]);
					#endif
				}
			} 
		}
		catch (boost::system::system_error ex)
		{
			printf("An exception was thrown: %s", ex.what());
		}
    }
    laser.close();
    return 0;
}

