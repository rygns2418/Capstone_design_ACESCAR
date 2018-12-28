#include <unistd.h>
#include "ros/ros.h"
#include "control_esc_v3/MsgControl.h"
#include <stdio.h>
#include <cstdio>
#include <cstdlib>
#include <pigpio.h>
#include <wiringPi.h>
#include <pthread.h>

#define __DEBUG__
//#define __NO_GO__
#define DELAY 0.05

int start = 0;


int receive_degree=-90;
int stop_signal = 0;
int back_signal = 0;
int corner_signal = 0;

void m_stop();
void m_forward(int speed);
void m_backward();
int PID_function();
void *t_function();
void *t1_function();




void m_backward()
{
	if(gpioInitialise()<0)
	{
		#ifdef __DEBUG__
		printf("library err\n");
		#endif
		return;
	}
	gpioSetMode(17,PI_OUTPUT);
	gpioSetPWMfrequency(17,40);
	gpioSetPWMrange(17,4000);

	//time_sleep(0.05);
	gpioPWM(17, 240); // 6% 듀티비 인가 (240/4000)
	time_sleep(DELAY); //필수 시간딜레이
	gpioPWM(17, 230); // 후진 신호 인가
	time_sleep(DELAY); // 필수 타임 딜레이
	gpioPWM(17,0); // 신호 끄기
	time_sleep(DELAY);

	///이전 코드에서 0V 인가 뒤 똑같은 후진동작 실행
	gpioPWM(17,240);
	time_sleep(DELAY);
	gpioPWM(17,230);
	time_sleep(DELAY);

	return;
}


void msgCallback(const control_esc_v3::MsgControl::ConstPtr& msg);

void *t_function(void *args)
{
	unsigned int rateTimer;
	unsigned int now;
	unsigned int controlPeriod = 1000;//1ms

	while (1)
	{
		if(receive_degree==-90)
	  	{
			while(1)
		   	{
	         		if(receive_degree!=-90)
    	      			{
			      		m_stop();
					time_sleep(2);
					#ifndef __NO_GO__
	      				m_forward(260);
					time_sleep(0.4);
	      			 	m_forward(250);
					#endif
	      				time_sleep(0.1);
					start = 1;
	        			break;
          			}
      			}
	  	}
		rateTimer = now = micros();
		while ((now - rateTimer) < controlPeriod)//loop untill 1ms
			now = micros();
		
	    	if(back_signal == 1)
    		{
    			gpioPWM(18,270);
    		}
    		else
    		{
     			 gpioPWM(18, PID_function());//PWM injection
    		}	
  
  		if(stop_signal==1)//1 = Stop Signal
    		{
      			while(1);
      			{
        			gpioPWM(18,270);
      			}
    		}
	}

}

void *t1_function(void *args)
{

	unsigned int rateTimer;
	unsigned int now;
	unsigned int controlPeriod = 1000;//1ms
 	int servoangle;

	if (start==0)
		while(1)
		{
			if(start==1)	break;
		}
  	while(1)
  	{
  		rateTimer = now = micros();
		while ((now - rateTimer) < controlPeriod)//loop untill 1ms
		now = micros();
    
         	if(stop_signal==1)//1 = Stop Signal
          	{
           		m_backward();
           		time_sleep(0.7);
             		while(1)
             		{
               			m_stop();
             		}
       		}
    		if(back_signal == 1) // backState
    		{
       			m_backward();
			time_sleep(0.7);
			m_stop();
			for(int i = 0;i<50;i++)
			{
				m_stop();

			}
			m_backward();

			while(1)
			{
				if(back_signal==0)
				break;
			}
    		}
    		else // goState
    		{
      			if (corner_signal == 1)
      			{   
				#ifndef __NO_GO__
				time_sleep(DELAY);
				m_forward(249);
				time_sleep(DELAY);
				#endif
				while(1)
				{
				        if(corner_signal == 0)
           				{
						#ifndef __NO_GO__
              					time_sleep(DELAY);
              					m_forward(255);
              					time_sleep(0.4);
						#endif
						break;
           				}
				}
      			}
      			else if(corner_signal == 0)
      			{
				#ifndef __NO_GO__
        			time_sleep(DELAY);
        			m_forward(250);
				while(1)
				{
					if(corner_signal != 0)
					break;
				}
				#endif
      			}
    		}
  	}
}


void msgCallback(const control_esc_v3::MsgControl::ConstPtr& msg)
{
	#ifdef __DEBUG__
	ROS_INFO("recieve msg = %d", msg->data);
	#endif

  	if( msg->data == 9999)// stop signal
    		stop_signal = 1;

  	if( msg->data == -5555) // backward signal
    		back_signal = 1;
  	else
    		back_signal = 0;
	
  	if( msg->data == 29) // corner signal
		corner_signal = 1;
  	else
    		corner_signal = 0;
    	
	receive_degree = msg->data;
	#ifdef __DEBUG__
	ROS_INFO("backsignal = %d\n",back_signal);
	#endif

}

void m_stop()
{
  	time_sleep(0.05);
	gpioPWM(17, 240); //natural
  	time_sleep(0.05);
	return;
}
void m_forward(int speed)
{
	if(gpioInitialise()<0)
	{
		#ifdef __DEBUG__
		printf("library err");
		#endif
		return;
	}

	gpioSetMode(17,PI_OUTPUT);
	gpioSetPWMfrequency(17,40);
	gpioSetPWMrange(17,4000);

	time_sleep(0.05);
	gpioPWM(17, 240);
	time_sleep(0.05);
	gpioPWM(17, speed);
	time_sleep(0.05);
	return;
}

int PID_function()
{
	int current_angle = receive_degree;//first degree from TX2
  	int AbsU;
  	int PWMvalue;
	AbsU = current_angle; // Absolute angle for control angle value
	
  	if(AbsU<30||AbsU>150) // if servo control angle is over the critical point
  	{
    		if(AbsU<30)
    		{
    			AbsU=30;
    		}
    		else
    		{
    			AbsU=150;
    		}
  	}

  	PWMvalue = 180 + AbsU; // PWMvalue for moving servo motor
  	return PWMvalue;
}



int main(int argc, char** argv)
{



	/////////////////////////
	ros::init(argc, argv, "esc_out_v3");
	ros::NodeHandle nh;
	ros::Subscriber ros_tutorial_sub = nh.subscribe("ros_msg", 100, msgCallback);

	////////////////////////////////


	if (wiringPiSetup() == -1) return 1;
	if (gpioInitialise()<0) return -1;

	//SERVO setting
	gpioSetMode(18, PI_OUTPUT);
	gpioSetPWMfrequency(18, 50);
	gpioSetPWMrange(18, 3600);
	
	//DC setting
	gpioSetMode(17, PI_OUTPUT); //use GPIO 17pin
	gpioSetPWMfrequency(17, 40); // set frequency 40Hz
	gpioSetPWMrange(17, 4000); // speed range transfer 255->4000
 
 


	/**************thread setting1 for ServoMotor**************************/
	pthread_t p_thread;
	int pid;
  	pid = pthread_create(&p_thread, NULL, t_function, NULL);
	if(pid<0)
  	{
		perror("thread create error");
		exit(1);
	}
	/******************************************************/

	/**************thread setting1 for DCMotor**************************/
	pthread_t p_thread1;
	int pid1;
  	pid1 = pthread_create(&p_thread1, NULL, t1_function, NULL);
	if(pid1<0)
 	{
		perror("thread create error");
		exit(1);
	}
	/******************************************************/


	ros::spin();
   
	return 0;
}
