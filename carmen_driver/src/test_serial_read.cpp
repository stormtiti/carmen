
#include <string>
#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "Motion.h"

using namespace carmen;

inline void printHex(char *data, int length)
{
  for (int i = 0; i < length; ++i) {
    printf("0x%.2X ", (unsigned)(unsigned char)data[i]);
  }
  printf("\n");
}


int main(int argc, char *argv[])
{
        SERIAL_INFO serialDsc;
        serialDsc.port = argv[1];
        serialDsc.baudrate = atoi(argv[2]);

/*	CarmenDriver readEngine;
	readEngine.configure(serialDsc.port, serialDsc.baudrate);
	readEngine.connect();
	unsigned char buffer[256]; 
	int byte= 	readEngine.read(buffer,256);
	std::cout <<"read byte: "<< byte <<std::endl;	
	printHex((char*)buffer,byte); */
        // init motion instance
        Motion::CreateInstance();
        Motion* motionEngine = Motion::GetInstance();

        // Configure serial port 
        motionEngine->Init(serialDsc);

	 // Triger reading thread
        if (motionEngine->connect())
        {	
		 motionEngine->KickoffReading();

	} 
	return 0;  
}
