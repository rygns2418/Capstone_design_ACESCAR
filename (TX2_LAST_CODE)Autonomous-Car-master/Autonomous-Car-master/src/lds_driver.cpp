#include "lds_driver.h"

#define __DEBUG__

namespace lds
{
LFCDLaser::LFCDLaser(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
  : port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
{
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

  // Below command is not required after firmware upgrade (2017.10)
  boost::asio::write(serial_, boost::asio::buffer("b", 1));  // start motor
}

LFCDLaser::~LFCDLaser()
{
  boost::asio::write(serial_, boost::asio::buffer("e", 1));  // stop motor
}

void LFCDLaser::poll(std::vector<double> &pnt, std::vector<double> &fntPnt, std::vector<double> &lr)
{
  uint8_t temp_char;
  uint8_t start_count = 0;
  bool got_scan = false;
  boost::array<uint8_t, 2520> raw_bytes;
  uint8_t good_sets = 0;
  uint32_t motor_speed = 0;
  rpms=0;
  int index;

  while (!shutting_down_ && !got_scan)
  {
    // Wait until first data sync of frame: 0xFA, 0xA0
    boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count],1));

    if(start_count == 0)
    {
      if(raw_bytes[start_count] == 0xFA)
      {
        start_count = 1;
      }
    }
    else if(start_count == 1)
    {
      if(raw_bytes[start_count] == 0xA0)
      {
        start_count = 0;

        // Now that entire start sequence has been found, read in the rest of the message
        got_scan = true;

        boost::asio::read(serial_,boost::asio::buffer(&raw_bytes[2], 2518));

        // scan->angle_min = 0.0;
        // scan->angle_max = 2.0*M_PI;
        // scan->angle_increment = (2.0*M_PI/360.0);
        // scan->range_min = 0.12;
        // scan->range_max = 3.5;
        // scan->ranges.resize(360);
        // scan->intensities.resize(360);

        //read data in sets of 6
        for(uint16_t i = 0; i < raw_bytes.size(); i=i+42)
        {
          if(raw_bytes[i] == 0xFA && raw_bytes[i+1] == (0xA0 + i / 42)) //&& CRC check
          {
            good_sets++;
            motor_speed += (raw_bytes[i+3] << 8) + raw_bytes[i+2]; //accumulate count for avg. time increment
            rpms=(raw_bytes[i+3]<<8|raw_bytes[i+2])/10;

            for(uint16_t j = i+4; j < i+40; j=j+6)
            {
              index = 6*(i/42) + (j-4-i)/6;

              // Four bytes per reading
              uint8_t byte0 = raw_bytes[j];
              uint8_t byte1 = raw_bytes[j+1];
              uint8_t byte2 = raw_bytes[j+2];
              uint8_t byte3 = raw_bytes[j+3];

              // Remaining bits are the range in mm
              uint16_t intensity = (byte1 << 8) + byte0;

              // Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
              // uint16_t intensity = (byte3 << 8) + byte2;
              uint16_t range = (byte3 << 8) + byte2;

              // scan->ranges[359-index] = range / 1000.0;
              // scan->intensities[359-index] = intensity;
              
              //##########need code freom here##########
              // degree = 0  is front
              // degree = 90 is left
              // degree = 270 is right
              
              int degree = 359 - index;
              double distance = range / 1000.0;
              if(distance < 0.000001)
			  {
				   switch(degree)
					{					
						case 60:
							distance = 1000;
							break;
						case 25:
							distance = 2000;
							break;
						case 0:
							distance = 3000;
							break;
						case 335:
							distance = 4000;
							break;
						case 300:
							distance = 5000;
							break;
						default:
							distance = 100;
							break;
					}
			  }	
				//distance =100;
              //get info 60,30,0,330,300
              //get info 20,10,0,350,340 (need to sin function)
              
              switch(degree)
              {
					case 90:
						lr[0] = distance;
						#ifdef __DEBUG__
						printf ("l=%f\n", distance);
						#endif
						break;				  
					case 60:
						pnt[0] = distance;
						#ifdef __DEBUG__
						printf ("a=%f\n", distance);
						#endif
						break;
					case 25:
						pnt[1] = distance;
						#ifdef __DEBUG__
						printf ("b=%f\n", distance);
						#endif
						break;
					case 0:
						pnt[2] = distance;
						fntPnt[2] = distance;
						#ifdef __DEBUG__
						printf ("c=%f\n", distance);
						#endif
						break;
					case 335:
						pnt[3] = distance;
						#ifdef __DEBUG__
						printf ("d=%f\n", distance);
						#endif
						break;
					case 300:
						pnt[4] = distance;
						#ifdef __DEBUG__
						printf ("e=%f\n", distance);
						#endif
						break;
					case 270:
						lr[1] = distance;
						#ifdef __DEBUG__
						printf ("r=%f\n", distance);
						#endif
						break;
					case 10:
						fntPnt[0] = (distance)*sin(70*3.1416/180);
						#ifdef __DEBUG__
						printf ("f1=%f\n", distance);
						#endif
						break;
					case 5:
						fntPnt[1] = (distance)*sin(80*3.1416/180);
						#ifdef __DEBUG__
						printf ("f2=%f\n", distance);
						#endif
						break;
					case 355:
						fntPnt[3] = (distance)*sin(80*3.1416/180);
						#ifdef __DEBUG__
						printf ("f4=%f\n", distance);
						#endif
						break;
					case 350:
						fntPnt[4] = (distance)*sin(70*3.1416/180);
						#ifdef __DEBUG__
						printf ("f5=%f\n\n", distance);
						#endif
						break;
			  }
              
              
				
			  //##########until here##########
            }
          }
        }

        // scan->time_increment = motor_speed/good_sets/1e8;
      }
      else
      {
        start_count = 0;
      }
    }
  }
}
}
