
#include <string>
#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "Motion.h"

using namespace carmen;

void processMotion(void* para, UInt8 lp, UInt8 li, UInt8 ld, UInt8 rp, UInt8 ri, UInt8 rd)
{
	std::cout<<">>>>>> Output PID >>>>>>>>>>"<<std::endl;
	std::cout<<"-------left motor PID: "<<lp<<","<<li<<","<<ld<<std::endl;
	std::cout<<"-------right motor PID: "<<rp<<","<<ri<<","<<rd<<std::endl;
}

void processEncoderAbs(void* para, Float32* lv, Float32* rv, Float32* ls, Float32* rs, Float32 elaspe)
{
	std::cout<<">>>>>> Output encoder absolute value >>>>>>>>>>"<<std::endl;
	std::cout<<"-------left motor: v and s "<<*lv<<","<<*ls<<std::endl;
	std::cout<<"-------right motor v and s: "<<*rv<<","<<*rs<<std::endl;
	std::cout<<"-------Time stamp: "<<elaspe<<std::endl;
}
void processSonar(void* para, Float32* value, SInt32 channel)
{
	std::cout<<">>>>>> Output sonar value >>>>>>>>>>"<<std::endl;
	std::cout<<"-------Sonar channel: "<< channel <<" value: "<<*value<<std::endl;
}
void processPsd(void* para, Float32* value, SInt32 channel)
{
	std::cout<<">>>>>> Output psd value >>>>>>>>>>"<<std::endl;
	std::cout<<"-------psd channel: "<< channel <<" value: "<<*value<<std::endl;
}
void processEncoder(void* para, Float32 value[4])
{
	std::cout<<">>>>>> Output encoder relative value >>>>>>>>>>"<<std::endl;
	std::cout<<"-------Left motor distance: "<< value[0] <<" Left motor distance coordinate: "<<value[2]<<std::endl;
	std::cout<<"-------Right motor distance: "<< value[1] <<" Right motor distance coordinate: "<<value[3]<<std::endl;
}
void processCheck(void* para, UInt8 id, UInt16 channel, UInt8 status)
{
	std::cout<<">>>>>> Output check status >>>>>>>>>>"<<std::endl;
	switch (id)
	{
		case ProtocolHeader::MOTION :
			{
				if (channel == 1)
				{
					switch (status)
					{
						case 0x01:
						std::cout<< "-------left motor checking!-------"<<std::endl;
						break;
						case 0x02:
						std::cout<< "-------left motor fault!-------"<<std::endl;
						break;
						case 0x04:
						std::cout<< "-------left motor working!-------"<<std::endl;
						break;
					}
				}

				if (channel == 9)
				{
					switch (status)
					{
						case 0x01:
						std::cout<< "-------right motor checking!-------"<<std::endl;
						break;
						case 0x02:
						std::cout<< "-------right motor fault!-------"<<std::endl;
						break;
						case 0x04:
						std::cout<< "-------right motor working!-------"<<std::endl;
						break;
					}
				}
				break;
			}

		case ProtocolHeader::SONAR :
		{
			switch (status)
			{
				case 0x01:
				std::cout<<"-------Sonar channel: "<<channel<<" checking!"<<std::endl;
				break;
				case 0x02:
				std::cout<<"-------Sonar channel: "<<channel<<" fault"<<std::endl;;
				break;
				case 0x04:
				std::cout<<"-------Sonar channel: "<<channel<<" working!"<<std::endl;;
				break;
			}
			break;
		}

		case ProtocolHeader::PSD :
		{
			switch (status)
			{
				case 0x01:
				std::cout<<"-------PSD channel: "<<channel<<" checking!"<<std::endl;
				break;
				case 0x02:
				std::cout<<"-------PSD channel: "<<channel<<" fault"<<std::endl;;
				break;
				case 0x04:
				std::cout<<"-------PSD channel: "<<channel<<" working!"<<std::endl;;
				break;
			}
			break;
		}
	}
}


/******
*	Running Examples:
*	carmen_example <serial port> <baudrate> <mode> <reading flag> <direction> 
*    			seriial port: /dev/ttyS1  
*				baudrate:	  19200
				mode:	the mode corresponding to inginious protocol
						0x01	move robot with velocity closed-loop
						0x02    move robot with postion closed-loop
						0x03    get sonar value
						0x05	get psd value
						0x0E(14)    get absolute encoder value
						0x0F(15)	get relative encoder and PID value
						0x11(17)    self check 
				reading flag: 
						1: Enable reading serial
						0: Disable reading serial
				direction: 
						1: move forward 
						0: move backward
				channel:        sonar and psd channel  (0--15)~(1--16)    
*/
void print_usage() 
{
  std::cout << "Usage: " << std::endl;
  std::cout << "       carmen_example <serial port> < baudrate> <mode> <reading flag> <direction>"<< std::endl;
  std::cout << "Examples:" << std::endl;
  std::cout << "       carmen_example /dev/ttyS1 19200 1 0 1" << std::endl;
}

int main(int argc, char *argv[]) 
{
	if (argc < 6) 
	{
    	print_usage();
    	return 0;
  	}

  	SERIAL_INFO serialDsc;
  	serialDsc.port = argv[1];
  	serialDsc.baudrate = atoi(argv[2]);
	
  	// init motion instance
  	Motion::CreateInstance();
  	Motion* motionEngine = Motion::GetInstance();

  	// Configure serial port 
  	motionEngine->Init(serialDsc);

  	// bind callback function for data processing
  	void* paramer;
  	motionEngine->SetMotionProc(processMotion,paramer);
  	motionEngine->SetSonarProc(processSonar,paramer);
  	motionEngine->SetPsdProc(processPsd,paramer);
  	motionEngine->SetEncoderProc(processEncoder,paramer);
  	motionEngine->SetEncoderAbsProc(processEncoderAbs,paramer);
  	motionEngine->SetCheckProc(processCheck,paramer);
	
 	std::cout<< "mode = "<< atoi(argv[3])<<std::endl;
  	// Triger reading thread
  	if (motionEngine->connect())
  	{
		switch(atoi(argv[3]))
		{
			case 0x01:     //test movement command, velocity closed-loop
				// left velocity: 0.1     left accerleration: 0.1
	            // right velocity: 0.1    left accerleration: 0.1
				if (atoi(argv[5]))
				{
					motionEngine->Drive(VEL, DEFAULT, 0.1, 0,0,0,0.1,0,0.1,0,0,0,0.1,0);	
				}
				else
				{
					motionEngine->Drive(VEL, DEFAULT,-0.1, 0,0,0,0.1,0,-0.1,0,0,0,0.1,0);	
				}
				break;
			case 0x02:     //test movement command, postion closed-loop
			 	// with position closed-loop  v:0.1 a:0.1 s=1
				if (atoi(argv[5]))
				{
					motionEngine->Drive(POS, DEFAULT, 0.1, 0,0,0,0.1,2,0.1,0,0,0,0.1,2);
				}
				else
				{
					motionEngine->Drive(POS, DEFAULT, -0.1, 0,0,0,0.1,2,-0.1,0,0,0,0.1,2);	
				}
            			break;
			case 0x03:     //test sonar 
				motionEngine->GetSonar(atoi(argv[6]),true);
			break;
			case ProtocolHeader::PSD:     //test psd
  				motionEngine->GetPsd(1,true);
  				break;
  			case ProtocolHeader::ENCODERDIS:       //test relative encoder
  				motionEngine->GetEncoderIncre();
  				break; 	
  			case ProtocolHeader::CHECK:            // self check
  				motionEngine->SelfCheck(true);
  				break;
  			case 0x0E:
  				motionEngine->GetMotion(char(0x01),true);  // get PID value 
  				break;
  			case 0x0F:
  				motionEngine->GetMotion(0,true);  // get absolute encoder value
  				break;
		}     

		if (atoi(argv[4]))       // open read serial port
		{
			motionEngine->KickoffReading();	 
		}
		          
		//clean up device and close the serial port
  		motionEngine->End();
  	}
  	else
  	{
  		LOG("The serial port isn't connected, please check!");
  	}
  		
  	return 0;
}
