#ifndef _MOTION_H_
#define _MOTION_H_

#include "boost/function.hpp"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include "GlobalDef.h"
#include "carmen_driver.h"
#include <iostream>     // std::cout
#include <algorithm>    // std::max
#include <exception>    // std::exception

#ifndef DEBUG 
#define DEBUG 0  // set debug mode
#endif

namespace carmen 
{

	/*!
	* A structure to represent the protocol header of control motherboard 	
	*/
	struct ProtocolHeader
	{

		UInt8 bytes;

		/*!
		*	The enumeration of device ID
		*/
		enum      
		{
			MOTION = 0x01,
			SONAR  = 0x03,
			PSD    = 0x05,
			DI     = 0x07,
			DO     = 0x09,
			DA     = 0x0B,
	        CHECK  = 0x11,        // self check 
			ENCODERDIS = 0x17    // relative encoder value
		};

		/*!
		*	The enumeration of enable toggle
		*/
		typedef enum 
		{
			ENABLE  = 0x01,
			DISABLE = 0x00 
		}ENABLE_TOGGLE;
	};

	
	
	/*!
	*	The enumeration of drive mode
	*/
	typedef enum 
	{
		VEL = 0x01,	/**< speed closure mode*/
		POS = 0x02,	/**< distance closure mode*/
		COOPERATE = 0x04,/**< fixme why*/
		SAVEPID = 0x08	/**< save PID mode*/
	}DRIVE_MODE;

	/*!
	*	The enumeration of the PID mode 
	*/
	typedef enum
	{
		DEFAULT = 0x01,	/**< default PID */
		ONLINE = 0x02,	/**< online debug PID */
		LOAD = 0x04		/**< load PID */
	}PID_MODE;

	/*!
	*	The structure of serial port information
	*/
	typedef struct  
	{
		std::string port; /**< the serial port address like 'COM1' on Windows and '/dev/ttyS1' on Linux.*/
		UInt32      baudrate;/**<  the baudrate of serial port */
	}SERIAL_INFO;


// define callback function
typedef boost::function<void(void*, UInt8,UInt8,UInt8,UInt8,UInt8,UInt8)> MotionProc;
typedef boost::function<void(void*, Float32*, Float32*, Float32*, Float32*, Float32)> EncoderAbsProc;
typedef boost::function<void(void*, Float32*, SInt32)> SonarProc;
typedef boost::function<void(void*, Float32*, SInt32)> PsdProc;
typedef boost::function<void(void*, Float32 value[])> EncoderProc;
typedef boost::function<void(void*, UInt8, UInt16, UInt8)> SelfCheckProc;

// define information print callback
typedef boost::function<void(const std::string&)> LogMsgCallback;
typedef boost::function<void(const std::exception&)> ExceptionCallback;

/*!
 * Class that provides control interface for carmen including motion control and sensor data acquasition.
 */
class Motion
{
public:

	enum { SONAR_NUM = 16 };
	enum { PSD_NUM = 16 };
	enum { ENCODER_NUM = 2};
	enum { DI_BITS = 16 };
	enum { DO_BITS = 16 };
	enum { DA_NUM = 6 };


    static void CreateInstance();
	static Motion* GetInstance();
	static void DeleteInstance();

	Motion(void);
	virtual ~Motion(void);

	/*!
   * Configure the serial port and initialize some variables. 
   * otherwise it remains closed until serial::Serial::open is called.
   * @param[in] desc the description of serial port
   * @return Returns true if the serial port is open, false otherwise
   */
	void Init(const SERIAL_INFO& desc);

	/*!
    *	Connect serial port.If the connection is done, start thread for reading serial port
    */
    void KickoffReading();

	/*!
	* Unenable controller motherboard and then close the serial port
	* @return none 
	*/
	void End();

	/*!
	*	Enable the motion functionality of controller motherboard
	*	@param[in]	mode the option for determining which motion functionality. 1: return PID valure 0: return absolute encoder value	
	*	@param[in]	enable  Option toggle, true for enable and false for unenable
	*	@return none
	*/
	void GetMotion(UInt16 mode, bool enable);

	/*!
	*	Disable motion functionality 
	*	@return none
	*/
	void DisableMotion();

	/*!
	*	The core function for moving the carmen robot.  
	*	@param[in]	ModeSelect the motion mode. 
								DRIVE_MODE::VEL  : PID velocity closure
								DRIVE_MODE::POS  : PID position closure
								DRIVE_MODE::SAVEPID : Save PID value
		@param[in]	PidSelect	PID value setting 
								PID_MODE::DEFAULT : the default PID value
								PID_MODE::ONLINE  :  online debugging PID
								PID_MODE::LOAD    :	use the PID value which is stored in memory
		@param[in]	lv 			Setting the velocity of left motor 	(m/s)
		@param[in]	lKp			Setting the proportion value of PID
		@param[in]	lKi			Setting the integral value of PID
		@param[in]	lKd 		Setting the derivative value of PID
		@param[in]	la 			Setting the acceleration vaule of left motor 
		@param[in]	ls          Setting the distance value of left motor   (m)
		@param[in]	rv 			Setting the velocity of right motor 	(m/s)
		@param[in]	rKp			Setting the proportion value of PID
		@param[in]	rKi			Setting the integral value of PID
		@param[in]	rKd 		Setting the derivative value of PID
		@param[in]	ra 			Setting the acceleration vaule of right motor 
		@param[in]	rs          Setting the distance value of right motor   (m)
		
		@return void	
	*/
	void Drive(SInt16 ModeSelect, SInt16 PidSelect,
				Float32 lv, UInt32 lKp, UInt32 lKi, UInt32 lKd,Float32 la, Float32 ls,
				Float32 rv, UInt32 rKp, UInt32 rKi, UInt32 rKd,Float32 ra, Float32 rs);

	/*!
	*	Send command to controller motherboard for getting the relative distance of the encode
	*/
    void GetEncoderIncre();

    /*!
	*	Send command to controller motherboard for closing encoder   
	*/
    void CloseEncoderEnable();

    /*!
	*	Send command to controller motherboard for getting the sonar value 
	*	
	*	@param[in]	channel The channel port 
		@param[in]	enable  Set true for trigerring action,otherwise false
	*/
    void GetSonar(UInt16  channel, Bool enable = true);

    /*!
	*	Send command to controller motherboard for closing sonar  
	*/
    void CloseSonarEnable();

    /*!
	*	Send command to controller motherboard for getting the PSD value 
	*	
	*	@param[in]	channel The channel port [0~15] 
		@param[in]	enable  Set true for trigerring action,otherwise false
	*/
    void GetPsd(UInt16 channel, bool enable = true);

    /*!
    *	Send command to control DO channel
    *	@param channel  The channel number [1~16]
    *	@param enable   True or false
    */
    void SetDO( UInt16 channel ,bool enable);

    /*!
    *	Send command to get DI channel
    *	@param channel  The channel number [1-16]
    *	@param enable   True or false
    */
    void GetDI(  UInt16 channel, bool enable );

    /*!
    *	Send command to output DA 
    *	@param channel  The channel number 
    *	@param vaule    The outputed value
    *	@param enable   True or false
    */
    void SetDA( UInt8 channel ,UInt8 value,bool enable);

    /*!
    *	Send command to check the carmen robot
    *	@param enable   Enable or not check process
    */
    void SelfCheck(bool enable);

    /*!
    *	Send command to controller motherboard for robot self checking
    *
    *	@param[in]	enable Set true for trigerring self checking , otherwise false
    */
   // void GetCheckStatus(bool enable);

    /**!
  	*	Call serial engine to connect the controller card
  	*/
	bool connect();


    void SetMotionProc(MotionProc callback, void*);
    void SetSonarProc(SonarProc callback,void*);
    void SetPsdProc(PsdProc callback,void*); 
    void SetEncoderAbsProc(EncoderAbsProc callback, void*);
    void SetEncoderProc(EncoderProc callback, void*);
    void SetCheckProc(SelfCheckProc callback, void*);


protected:	

	/*!
	*	Authenticate the received packet to make sure it's an availabe packet
	*	@param[in] frame The received packet
	*   @return True if the packet is available,otherwise false
	*/
	bool AuthenticatePacket(const Packet& packet);


private:

	/*!
	* Thread function for grabbing a packet from serial port and then parse the packet for further operation.
	* The funcition is binded by boost::thread library 
	*/
  	void ReadContinuously_();
  	void ExecuteCallbacks_();

  	/*!
  	*	Start a thread for continously reading the serial port
  	*/
  	void StartReadingContinuously_();

  	/*!
  	*	Stop read serial port and disable some related flag variables
  	*/
  	void StopReadingContinuously_();

  	bool continuously_reading_;    /**< The flag indicatinig whether read or not */
  	boost::thread read_thread_;    /**< The boost thread handle */

  	/*!
  	*	Process the packet builded by Motion::BuildPacket function
  	*/
  	bool ProcessPacket_(Packet &packet);

  	/*!
  	*	Parse the packet according to carmen serial communication protocal
  	*	@param packet the received data packet
  	*	@return true or false 
  	*/
  	bool ParsePacket_(const Packet& packet);

  	/*!
  	*	Send command to disable opened device
  	*	@param mode the ID corresponding to device 
  	*/
  	void sendCleanup_(char mode);



  	/*!
  	*	
  	*/
	inline Float32 Time2Distance( UInt16 timer );
	inline Float32 Voltage2Distance(UInt16 voltage);

private:
	static Motion* s_instance;

	carmen::CarmenDriver *m_serialEngine;
	bool m_ConnectStatus;     // connect flag of serial port

	void Cleanup();
	

	void* m_pMotionParam;

	// for  sonar mode
	void*   m_pSonarParam;

	// for user callback
	void*      m_pPsdParam;

 
	void*      m_pEncoderParam;

	
	void* m_pCheckParam;


	void* m_pEncoderAbsParam;

	std::vector<char> m_EnabledIDStatus;   // store the device ID enabled


	// Callback function for dealing with received data

	MotionProc m_MotionProc;
	SonarProc  m_SonarProc;
	PsdProc    m_PsdProc;
	EncoderProc m_EncoderProc;
	EncoderAbsProc m_EncoderAbsProc;
	SelfCheckProc  m_CheckProc;

	LogMsgCallback debug_, info_, error_;
	ExceptionCallback handle_exception_;


};

} //namespace carmen 


#endif
