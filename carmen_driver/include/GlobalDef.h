#ifndef _GLOBAL_DEF_H_
#define _GLOBAL_DEF_H_

#include <stdio.h> 
#include <exception>
#include <stdint.h>
#include <vector>
#include <sstream>

/****define type*****/

typedef unsigned char      UInt8;
typedef short int          SInt16;
typedef unsigned short int UInt16;
typedef int                SInt32;
typedef unsigned int       UInt32;
typedef float              Float32;
typedef double             Float64;
typedef bool               Bool;



/******define protocol header******/
#define CMD_FRAMEHEAD1 0xA5 
#define CMD_FRAMEHEAD2 0x5A
#define CMD_FRAMEEND1 0x5A
#define CMD_FRAMEEND2 0XA5

#define RES_FRAMEHEAD1 0x5A 
#define RES_FRAMEHEAD2 0xA5
#define RES_FRAMEEND1 0xB5
#define RES_FRAMEEND2 0X5B


/*****define command length******/
#define MOTION_CMD_LENGTH 7
#define SONAR_CMD_LENGTH  8
#define PSD_CMD_LENGTH    8
#define ENCODER_CMD_LENGTH 7
#define CHECK_CMD_LENGTH  7
#define DI_CMD_LENGTH     8
#define DO_CMD_LENGTH     8
#define DA_CMD_LENGTH     8
#define DISABLE_MOTION_CMD_LENGTH 6

/********define robot dimensition**********/
const Float32 DIAMETER = 0.194f;
const Float32 PI = 3.1415926f;
const UInt32 BUFFER_SIZE = 256;




// This variable defines the minimal number of buffer when reading serial port. 
// If the buffer is less than the value, try to read serial port for filling the buffer
// The value is used in function CarmenDriver::BuildPacket()
#define MINIMIZE_BUFFER_NUM 20


/*****define exception handling macro ****/

// ClassName - The name of the exception class
// Prefix    - The common prefix
// Message   - The default message, if one is not specified at Throw
// The message takes the form of:
// ClassName occurred at line # of file `<file>`: Prefix+Message
#define DEFINE_EXCEPTION(ClassName, Prefix, Message) \
  class ClassName : public std::exception { \
    void operator=(const ClassName &); \
    const ClassName & operator=( ClassName ); \
    const char * what_; \
    const int id_; \
  public: \
    ClassName(const char * file, const int ln, \
              const char * msg = Message, const int id = -1) : id_(id) { \
      std::stringstream ss; \
      ss << #ClassName " occurred at line " << ln \
         << " of `" << file << "`: " << Prefix << msg; \
      what_ = ss.str().c_str(); \
    } \
    virtual const char* what () const throw () { return what_; } \
    const int error_number() { return this->id_; } \
  };


#define CARMEN_THROW(ExceptionClass) \
  throw ExceptionClass(__FILE__, __LINE__)

#define CARMEN_THROW_MSG(ExceptionClass, Message) throw ExceptionClass(__FILE__, __LINE__, (Message) )

#define CARMEN_THROW_MSG_AND_ID(ExceptionClass, Message, Id) \
  throw ExceptionClass(__FILE__, __LINE__, (Message), (Id) )

/***********define debug output macro**************/
/*  This macro can be used to print debug information. 
*The benefit of the macro is that it can precisely tell you where the debug information occurs
* Usage: log(">>> test...")
* Output: xxxx/proj.ios_mac/Classes/IntroScene.cpp][gotoNextScene][Line 58] >>> test...
*/
#define LOG(...) {\
    char str[100];\
    sprintf(str, __VA_ARGS__);\
    std::cout << "[" << __FILE__ << "][" << __FUNCTION__ << "][Line " << __LINE__ << "] " << ">>> "<< str << std::endl;\
    }

DEFINE_EXCEPTION(ConnectionFailedException, "Error connecting to the "
  "Carmen: ", "Unspecified");

DEFINE_EXCEPTION(ReadFailedException, "Error reading from the Carmen: ",
  "Unspecified");

DEFINE_EXCEPTION(WriteFailedException, "Error writing to the Carmen: ",
  "Unspecified");

DEFINE_EXCEPTION(MoveFailedException, "Error moving the Carmen: ",
  "Unspecified");

DEFINE_EXCEPTION(ConfigurationException, "Error configuring the Carmen: ",
  "Unspecified");

DEFINE_EXCEPTION(PacketRetrievalException, "Error retrieving a packet from the"
  " Carmen: ", "Unspecified");


#endif
