
#include "Motion.h"
#include <cstdio>
//#include <cmath>
#include "math.h"

using namespace carmen;

Motion* Motion::s_instance = 0;

inline void defaultErrorMsgCallback(const std::string &msg)
{
  std::cerr << "Carmen Robot Error: " << msg << std::endl;
}

inline void defaultDebugMsgCallback(const std::string &msg)
{
  std::cerr << "Carmen Debug: " << msg << std::endl;
}

inline void defaultInfoMsgCallback(const std::string &msg)
{
  std::cerr << "Carmen Info: " << msg << std::endl;
}

inline void defaultExceptionCallback(const std::exception &error)
{
  std::cerr << "Carmen Unhandled Exception: " << error.what()
            << std::endl;
}

Motion::Motion(void) 
:m_SonarProc(0)																																							 
, m_pSonarParam(0)
, m_pPsdParam(0)
, m_PsdProc(0)
, m_EncoderProc(0)
, m_pEncoderParam(0)
, m_MotionProc(0)
, m_pMotionParam(0)
, m_EncoderAbsProc(0)
, m_pEncoderAbsParam(0)
, m_ConnectStatus(false)
, error_(defaultErrorMsgCallback)
, debug_(defaultDebugMsgCallback)
, info_(defaultInfoMsgCallback)
,handle_exception_(defaultExceptionCallback)
{
	m_serialEngine = new carmen::CarmenDriver();
}

Motion::~Motion(void)
{
	// stop reading thread
	if (this->continuously_reading_) 
	{
    	this->StopReadingContinuously_();
  	}

  	// delete carmen driver class
	CarmenDriver * ptr = (CarmenDriver *)(this->m_serialEngine);
    delete ptr;
}



void Motion::CreateInstance()
{
	if ( 0 == s_instance )
	{
		s_instance = new Motion;
	}
}

Motion* Motion::GetInstance()
{
	return s_instance;
}

void Motion::DeleteInstance()
{
	if ( s_instance != 0 )
	{
		delete s_instance;
		s_instance = 0;
	}
}

void Motion::Init(const carmen::SERIAL_INFO& desc)
{
	// configure serial port
	m_serialEngine->configure(desc.port,desc.baudrate);	
	#ifdef DEBUG
	LOG("complete serial port configuration!");
	#endif

	// disable all functionality of controllor motherboard
	#ifdef DEBUG
	LOG("Complete carmen cleanup!");
	#endif
}

bool Motion::connect()
{
	 m_serialEngine->connect();
	 if (m_serialEngine->isConnected())
	 {
	 	#ifdef DEBUG
		LOG("Complete connecting Serial port!");
		#endif
	 	m_ConnectStatus = true;
        	Cleanup();
	 	return true;
	 }
	 else
	 {
		return false;
	 }

}

void Motion::KickoffReading()
{
	if (m_ConnectStatus)
	{
		// Kick off the read thread
		#ifdef DEBUG
		LOG("Continuously reading serial port.....");
		#endif
  		this->StartReadingContinuously_();
	}
}

void Motion::StartReadingContinuously_() {
  this->continuously_reading_ = true;
  this->read_thread_ =
    boost::thread(&Motion::ReadContinuously_, this);
 //this->read_thread_.join(); 
}

void Motion::StopReadingContinuously_()
{
  this->continuously_reading_ = false;
  this->read_thread_.join();

  this->m_serialEngine->cancel();
}

void Motion::ReadContinuously_() 
{
  Packet packet;
  while (this->continuously_reading_) 
  { 
    try 
    {
      if (this->m_serialEngine->BuildPacket(packet))
      {
      	if (this->ProcessPacket_(packet))
      	{
      		#ifdef DEBUG
      //		LOG("Complete parsing packet!");
      		#endif
      	}
      }  
    } 
    catch (PacketRetrievalException &e) 
    {
      if (e.error_number() == 3) // No packet received
        this->error_("No data from carmen robot...");
      else
        this->handle_exception_(e);
    }  

    // make thread sleep 200 milliseconds
    boost::this_thread::sleep(boost::posix_time::milliseconds(  5 ) );
  }
}

bool Motion::ProcessPacket_(Packet &packet)
{

  if (this->AuthenticatePacket(packet))
  {
  	return this->ParsePacket_(packet);
  }
  else
  {
  	 this->error_("The packet isn't right!");
  	 return false;
  }
}


void Motion::End()
{
	Cleanup();
	m_serialEngine->disconnect();
}



bool Motion::AuthenticatePacket(const Packet& packet)
{ 
	unsigned char module_id = packet[2]; 
	if (   module_id ==  ProtocolHeader::MOTION
			|| module_id ==  ProtocolHeader::SONAR
			|| module_id ==  ProtocolHeader::PSD
			|| module_id ==  ProtocolHeader::DI
			|| module_id ==  ProtocolHeader::DO
			|| module_id ==  ProtocolHeader::DA
			|| module_id ==  ProtocolHeader::CHECK
			|| module_id ==  ProtocolHeader::ENCODERDIS
		   )
		{
			
			return true;
		}

	return false;


}


bool Motion::ParsePacket_(const Packet& packet)
{
	
	switch ( packet[2] )
	{
	case ProtocolHeader::MOTION:
		{
			if ( packet[0]==0x5A 
				&& packet[1]==0xA5
				&& packet[3]==0x06
				&& packet[10]==0xB5
				&& packet[11]==0x5B) //PID
			{
				{
					if (m_MotionProc)
					{
						this->m_MotionProc(m_pMotionParam, packet[4],packet[5],packet[6],packet[7],packet[8],packet[9]);
					}
				}
			}
			else if (	packet[0]==0x5A
						&& packet[1]==0xA5
						&& packet[3]==0x11
						&& packet[21]==0xB5
						&& packet[22]==0x5B) // absolute movement of encoder
			{
				{
					Float32 lvelocity = 0,rvelocity = 0,ldistance = 0,rdistance = 0;
					if (packet[4]==0x00)
					{
						lvelocity = ((packet[6]<<8)| packet[5])*2.5f/1350.0f;
					}
					else if (packet[4]==0x02)
					{
						lvelocity = -(((packet[6]<<8) | packet[5])&0x7FFF)*2.5f/1350.0f;
					}
					if (packet[12]==0x00)
					{
						rvelocity = ((packet[14]<<8)|packet[13])*2.5f/1350.0f;
					}
					else if (packet[12]==0x02)
					{
						rvelocity = - (((packet[14]<<8)|packet[13])&0x7FFF)*2.5f/1350.0f;
					}
					if (packet[7]==0x00)
					{
						ldistance = ((packet[9]<<8|packet[8])*30000*19.2*PI*0.01)/66000+((packet[11]<<8|packet[10])*19.2*PI*0.01)/66000;
					}
					else if (packet[7]==0x02)
					{
						ldistance = -(((packet[9]<<8|packet[8])*30000*19.2*PI*0.01)/66000+(((packet[11]<<8|packet[10])&0x7FFFFFFF)*19.2*PI*0.01)/66000);
					}
					if (packet[15]==0x00)
					{
						rdistance = ((packet[17]<<8|packet[16])*30000*19.2*PI*0.01)/66000+((packet[19]<<8|packet[18])*19.2*PI*0.01)/66000;
					}
					else if (packet[15]==0x02)
					{
						rdistance = -(((packet[17]<<8|packet[16])*30000*19.2*PI*0.01)/66000+(((packet[19]<<8|packet[18])&0x7FFFFFFF)*19.2*PI*0.01)/66000);
					}
					if (m_EncoderAbsProc)
					{
						this->m_EncoderAbsProc(m_pEncoderAbsParam,&lvelocity,&rvelocity,&ldistance,&rdistance,5*packet[20]/1000.0f);
					}
				}
			}
		}
		break;
	case ProtocolHeader::SONAR:// sonar value
		{
			if ( packet[0]==0x5A
				&& packet[1]==0xA5
				&& packet[3]==0x04
				&& packet[8]==0xB5
				&& packet[9]==0x5B )
			{
				UInt16  channel = (packet[5]<<8) | packet[4];
				Float32 data = ( packet[7] << 8 ) | packet[6];

				Float32 distance;
				static Float32 value[16] = {0};
				#ifdef DEBUG
				LOG("preparing sonar parsing............");
				#endif
				for (SInt32 i=0; i<SONAR_NUM; i++ )
				{
					if (channel & (1<<i))
					{
						distance = Time2Distance(data);
						//distance[i] = Time2Distance(data);
						if (distance<0.2)
						{
							distance = value[i];
						}
						else
						{
							value[i] = distance;
						}
						if ( m_SonarProc )
						{	
							m_SonarProc( m_pSonarParam, &distance, i );
				#ifdef DEBUG
				LOG("preparing sonar parsing............");
				#endif
						}
					}
				}
			}
		}
		break;
	case ProtocolHeader::PSD:// psd value
		{
			if (packet[0]==0x5A
				&& packet[1]==0xA5
				&& packet[3]==0x04
				&& packet[8]==0xB5
				&& packet[9]==0x5B )
			{
				UInt16  channel = (packet[5]<<8) | packet[4];

				UInt16 data = ( packet[7] << 8 ) | packet[6];
			
				Float32 distance;
				for (UInt16 i = 0; i < PSD_NUM; i++ )
				{
					if (channel & (1<<i))
					{
						//distance = Time2Distance(data);
						distance = Voltage2Distance(data);
						if ( m_PsdProc )
						{	
							m_PsdProc( m_pPsdParam, &distance, i );
						}
					}
				}
			}
		}
		break;
	case ProtocolHeader::DI:
		break;
	case ProtocolHeader::DO:
		break;
	case ProtocolHeader::DA:
		
		break;
	case ProtocolHeader::CHECK:// self check
		{
			if (   packet[0]==0x5A
				&& packet[1]==0xA5
				&& packet[3]==0x04
				&& packet[8]==0xB5
				&& packet[9]==0x5B )
			{
				UInt16  channel = (packet[6]<<8) | packet[5];
                
				{

					for (UInt16 i=0; i <16; i++ )
					{
						if (channel & (1<<i))
						{
							m_CheckProc(m_pCheckParam, packet[4], i+1, packet[7]);
						}
					}
				}
			}
		}
		break;
	case ProtocolHeader::ENCODERDIS:// relative movement of encoder
		{
			if (   packet[0]==0x5A
				&& packet[1]==0xA5
				&& packet[3]==0x08
				&& packet[12]==0xB5
				&& packet[13]==0x5B)
			{
				{

					Float32 value[4]={0};
					if (packet[4]&0x80)
					{
						value[0] = -(((packet[4]&0x7F)<<8|packet[5])*30000+(packet[6]<<8|packet[7]))*PI*0.192/66000;
					}
					else
					{
						value[0] = ((packet[4]<<8|packet[5])*30000+(packet[6]<<8|packet[7]))*PI*0.192/66000;
					}
					if (packet[8]&0x80)
					{
						value[1] = -(((packet[8]&0x7F)<<8|packet[9])*30000+(packet[10]<<8|packet[11]))*PI*0.192/66000;
					}
					else
					{
						value[1] = ((packet[8]<<8|packet[9])*30000+(packet[10]<<8|packet[11]))*PI*0.192/66000;
					}
					value[2] = (value[0]+value[1])/2*cos((value[1]-value[0])/0.4);    // x y displacement
					value[3] = (value[0]+value[1])/2*sin((value[1]-value[0])/0.4);
					//out<<value[1]<<" "<<test<<endl;
					if (m_EncoderProc)
					{
						m_EncoderProc(m_pEncoderParam,value);
					}
				}
			}
		
		}
		break;
	}

	return true;
}


Float32 Motion::Time2Distance( UInt16 timer )
{
	const Float32 V = 343.2f;
	return 1.0f*timer*V/20000;
}

Float32 Motion::Voltage2Distance(UInt16 voltage)
{
	const Float32 coef = -1.1981f;
	Float32 temp = 2.5f* voltage/1023.0f;
	return 29.283f*pow((double)temp, (double)coef);
}


void Motion::sendCleanup_(char mode)
{
	char buff[6];
	buff[0] = (char)CMD_FRAMEHEAD1;
	buff[1] = (char)CMD_FRAMEHEAD2;
	buff[2] = (char)mode;
	buff[3] = (char)0x00;
	buff[4] = (char)CMD_FRAMEEND1;
	buff[5] = (char)CMD_FRAMEEND2;
	this->m_serialEngine->write((unsigned char*)buff, 6);
}
void Motion::Cleanup()
{
	char buff[6];
	memset(buff,0,sizeof(buff));
	if (m_EnabledIDStatus.size() == 0)
	{
		sendCleanup_((char)0x00);
		sendCleanup_((char)0x02);
		sendCleanup_((char)0x04);
		sendCleanup_((char)0x10);
	}
	else
	{
		for (int i = 0; i < m_EnabledIDStatus.size(); i++)
		{
			switch (m_EnabledIDStatus[i])
			{
				case ProtocolHeader::MOTION:
					sendCleanup_((char)0x00);
					break;
				case ProtocolHeader::SONAR:
					sendCleanup_((char)0x02);
					break;
				case ProtocolHeader::PSD:
					sendCleanup_((char)0x04);
					break;
				case ProtocolHeader::CHECK:
					sendCleanup_((char)0x10);
					break;
				case ProtocolHeader::ENCODERDIS:
					sendCleanup_((char)0x16);
					break;
				case 0x0F:
					sendCleanup_((char)0x00);
					break;
			}
			
		}
	 }
	m_EnabledIDStatus.clear();

}

void Motion::SetEncoderAbsProc(EncoderAbsProc proc,void* pParam)
{
	m_EncoderAbsProc = proc;
	m_pEncoderAbsParam = pParam;
}

void Motion::GetEncoderIncre()
{
	char buff[7];
	memset(buff,0,sizeof(buff));
	m_EnabledIDStatus.push_back(ProtocolHeader::ENCODERDIS);


	buff[0] = (char)CMD_FRAMEHEAD1;
	buff[1] = (char)CMD_FRAMEHEAD2;
	buff[2] = (char)ProtocolHeader::ENCODERDIS;
	buff[3] = 1;
	buff[4] = (char)0x01;
	buff[5] = (char)CMD_FRAMEEND1;
	buff[6] = (char)CMD_FRAMEEND2;
	this->m_serialEngine->write((unsigned char*)buff, ENCODER_CMD_LENGTH);
}

void Motion::Drive(SInt16 ModeSelect, SInt16 PidSelect,
				    Float32 lv, UInt32 lKp, UInt32 lKi, UInt32 lKd,Float32 la,Float32 ls,     // left motor: v P I D a s
				    Float32 rv, UInt32 rKp, UInt32 rKi, UInt32 rKd,Float32 ra,Float32 rs)     // right motor: v P I D a s
{
	UInt8 lvMode=0;
	UInt8 rvMode=0;
	SInt32 lhs=0;
	SInt32 lls=0;
	SInt32 rhs=0;
	SInt32 rls=0;
	UInt8 sum=0;
	lv = lv*1350.0/2.5f;
	rv = rv*1350.0/2.5f; 
	la = la*54.71f;
	ra = ra*54.71f;

	if (lv<0)
	{
		lv = -lv;
		lvMode = 0x09;
	}
	else
	{
		lvMode = 0x01;
	}
	if (rv<0)
	{
		rv = -rv;
		rvMode = 0x0A;
	}
	else
	{
		rvMode = 0x02;
	}
	lv = std::min(std::max(lv,0.0f),1350.0f);  // the maximum speed 0x0546
	rv = std::min(std::max(rv,0.0f),1350.0f);

	if (ls<0)
	{
		lhs = (int)(11.4583*(-ls)/PI);
		lls = (int)((11.4583*(-ls)/PI-lhs)*30000);
	}
	else
	{
		lhs = (int)(11.4583*(ls)/PI);
		lls = (int)((11.4583*(ls)/PI-lhs)*30000);
	}
	if (rs<0)
	{
		rhs = (int)(11.4583*(-rs)/PI);
		rls = (int)((11.4583*(-rs)/PI-rhs)*30000);
	}
	else
	{
		rhs = (int)(11.4583*(rs)/PI);
		rls = (int)((11.4583*(rs)/PI-rhs)*30000);
	}
	
    switch(ModeSelect)
	{
    case 1:
		sum = 10;
    	break;
	case 2:
        sum = 18;
		break;
	case 5:
		sum = 10;
		break;
	case 6:
		sum = 18;
		break;
	case 8:
		sum = 7;
		break;
    default:
		break;
    }
	switch(PidSelect)
	{
	case 1:
		break;
	case 2:
		if (ModeSelect!=8)
		{
			sum +=6;
		}
		break;
	case 4:
		break;
	default:
		break;
	}
    if (ModeSelect==1||ModeSelect==5)// velocity closed-loop mode
	{
		if(PidSelect!=2)   // not online debug. PID valuse don't need to be sent.
		{
			char buff1[16];
			memset(buff1,0,sizeof(buff1));
			buff1[0] = (char)CMD_FRAMEHEAD1;
			buff1[1] = (char)CMD_FRAMEHEAD2;
			buff1[2] = (char)0x01;
			buff1[3] = (char)sum;
			buff1[4] = (char)ModeSelect;
			buff1[5] = (char)PidSelect;
			buff1[6] = (char)lvMode;
			buff1[7] = (char)rvMode;
			buff1[8] = (char)la;
			buff1[9] = (char)ra;
			buff1[10] = ((UInt16)lv & 0xFF00)>>8;
			buff1[11] = (UInt16)lv & 0x00FF;
			buff1[12] = ((UInt16)rv & 0xFF00)>>8;
			buff1[13] = (UInt16)rv & 0x00FF;
			buff1[14] = (char)CMD_FRAMEEND1;
			buff1[15] = (char)CMD_FRAMEEND2;
			this->m_serialEngine->write((unsigned char*)buff1, 16);
		}
		else              // online debug PID. The PID value sould be sent via serial 
		{
			char buff1[22];
			memset(buff1,0,sizeof(buff1));
			buff1[0] = (char)CMD_FRAMEHEAD1;
			buff1[1] = (char)CMD_FRAMEHEAD2;
			buff1[2] = (char)0x01;
			buff1[3] = (char)sum;
			buff1[4] = (char)ModeSelect;
			buff1[5] = (char)PidSelect;
			buff1[6] = (char)lvMode;
			buff1[7] = (char)rvMode;
			buff1[8] = (char)la;
			buff1[9] = (char)ra;
			buff1[10] = ((UInt16)lv & 0xFF00)>>8;
			buff1[11] = (UInt16)lv & 0x00FF;
			buff1[12] = ((UInt16)rv & 0xFF00)>>8;
			buff1[13] = (UInt16)rv & 0x00FF;
			buff1[14] = (char)lKp;
			buff1[15] = (char)lKi;
			buff1[16] = (char)lKd;
			buff1[17] = (char)rKp;
			buff1[18] = (char)rKi;
			buff1[19] = (char)rKd;
			buff1[20] = (char)CMD_FRAMEEND1;
			buff1[21] = (char)CMD_FRAMEEND2;
			this->m_serialEngine->write((unsigned char*)buff1, 22);
		}
	}
	else if (ModeSelect==2||ModeSelect==6)// position closed-loop mode
	{
		if (PidSelect!=2)      // not online debug. PID valuse don't need to be sent.
		{
			char buff2[24];
			memset(buff2,0,sizeof(buff2));
			buff2[0] = (char)CMD_FRAMEHEAD1;
			buff2[1] = (char)CMD_FRAMEHEAD2;
			buff2[2] = (char)0x01;
			buff2[3] = (char)sum;
			buff2[4] = (char)ModeSelect;
			buff2[5] = (char)PidSelect;
			buff2[6] = (char)lvMode;
			buff2[7] = (char)rvMode;
			buff2[8] = (char)la;
			buff2[9] = (char)ra;
			buff2[10] = ((UInt16)lv & 0xFF00)>>8;
			buff2[11] = (UInt16)lv & 0x00FF;
			buff2[12] = ((UInt16)rv & 0xFF00)>>8;
			buff2[13] = (UInt16)rv & 0x00FF;
			if (ls<0)
			{
				buff2[14] = (lhs|0x8000)>>8;
				buff2[15] = lhs;
				buff2[16] = lls>>8;
				buff2[17] = lls;
			}
			else
			{
				buff2[14] = lhs>>8;
				buff2[15] = lhs;
				buff2[16] = lls>>8;
				buff2[17] = lls;
			}
			if (rs<0)
			{
				buff2[18] = (rhs|0x8000)>>8;
				buff2[19] = rhs;
				buff2[20] = rls>>8;
				buff2[21] = rls;
			}
			else
			{
				buff2[18] = rhs>>8;
				buff2[19] = rhs;
				buff2[20] = rls>>8;
				buff2[21] = rls;
			}
			buff2[22] = (char)CMD_FRAMEEND1;
			buff2[23] = (char)CMD_FRAMEEND2;
			this->m_serialEngine->write((unsigned char*)buff2, 24);
		}
		else              // Online debug PID. The PID value sould be sent via serial 
		{
			char buff2[30];
			memset(buff2,0,sizeof(buff2));
			buff2[0] = (char)CMD_FRAMEHEAD1;
			buff2[1] = (char)CMD_FRAMEHEAD2;
			buff2[2] = (char)0x01;
			buff2[3] = (char)sum;
			buff2[4] = (char)ModeSelect;
			buff2[5] = (char)PidSelect;
			buff2[6] = (char)lvMode;
			buff2[7] = (char)rvMode;
			buff2[8] = (char)la;
			buff2[9] = (char)ra;
			buff2[10] = ((UInt16)lv & 0xFF00)>>8;
			buff2[11] = (UInt16)lv & 0x00FF;
			buff2[12] = ((UInt16)rv & 0xFF00)>>8;
			buff2[13] = (UInt16)rv & 0x00FF;
			if (ls<0)
			{
				buff2[14] = (lhs|0x8000)>>8;
				buff2[15] = lhs;
				buff2[16] = lls>>8;
				buff2[17] = lls;
			}
			else
			{
				buff2[14] = lhs>>8;
				buff2[15] = lhs;
				buff2[16] = lls>>8;
				buff2[17] = lls;
			}
			if (rs<0)
			{
				buff2[18] = (rhs|0x8000)>>8;
				buff2[19] = rhs;
				buff2[20] = rls>>8;
				buff2[21] = rls;
			}
			else
			{
				buff2[18] = rhs>>8;
				buff2[19] = rhs;
				buff2[20] = rls>>8;
				buff2[21] = rls;
			}
			buff2[22] = (char)lKp;
			buff2[23] = (char)lKi;
			buff2[24] = (char)lKd;
			buff2[25] = (char)rKp;
			buff2[26] = (char)rKi;
			buff2[27] = (char)rKd;
			buff2[28] = (char)CMD_FRAMEEND1;
			buff2[29] = (char)CMD_FRAMEEND2;
			this->m_serialEngine->write((unsigned char*)buff2, 30);
		}
	}
    else if (ModeSelect==8)
	{
		char buff4[13];
		memset(buff4,0,sizeof(buff4));
		buff4[0] = (char)CMD_FRAMEHEAD1;
		buff4[1] = (char)CMD_FRAMEHEAD2;
		buff4[2] = (char)0x01;
		buff4[3] = (char)sum;
		buff4[4] = (char)ModeSelect;
		buff4[5] = (char)lKp;
		buff4[6] = (char)lKi;
		buff4[7] = (char)lKd;
		buff4[8] = (char)rKp;
		buff4[9] = (char)rKi;
		buff4[10] = (char)rKd;
		buff4[11] = (char)CMD_FRAMEEND1;
		buff4[12] = (char)CMD_FRAMEEND2;
		this->m_serialEngine->write((unsigned char*)buff4, 13);
	}
}


void Motion::GetMotion(UInt16 mode, bool enable)
{
 
    unsigned char buff[MOTION_CMD_LENGTH];
    UInt16 motionMode = 0;
	if (enable)
	{
		// first bit OR and then move "mode" bits toward left
		// 01: absolute encoder value    02: PID value
		motionMode|=1<<mode;
	}
    else
	{
		motionMode=0;
	}
	buff[0] = (char)CMD_FRAMEHEAD1;
	buff[1] = (char)CMD_FRAMEHEAD2;
	buff[2] = (char)0x0F;
	buff[3] = (char)0x01;
	buff[4] = motionMode;
	buff[5] = (char)CMD_FRAMEEND1;
	buff[6] = (char)CMD_FRAMEEND2;

	this->m_serialEngine->write((unsigned char*)buff, MOTION_CMD_LENGTH);
	m_EnabledIDStatus.push_back((char)0x0F);
}

void Motion::DisableMotion()
{

	unsigned char buff[DISABLE_MOTION_CMD_LENGTH];
	buff[0] = (char)CMD_FRAMEHEAD1;
	buff[1] = (char)CMD_FRAMEHEAD2;
	buff[2] = (char)0x00;
	buff[3] = (char)0x00;
	buff[5] = (char)CMD_FRAMEEND1;
	buff[6] = (char)CMD_FRAMEEND2;
	this->m_serialEngine->write((unsigned char*)buff, DISABLE_MOTION_CMD_LENGTH);
}

void Motion::SetMotionProc(MotionProc proc,void* pParam)
{
	m_MotionProc = proc;
	m_pMotionParam = pParam;
}
void Motion::SetSonarProc(SonarProc proc, void* pParam)
{
	m_SonarProc = proc;
	m_pSonarParam = pParam;
}

void Motion::GetSonar( UInt16  channel, bool enable )
{
	char buff[8];
	UInt16 sonarMode = 0;
        int mode = 0x02;
	memset(buff,0,sizeof(buff));
	
	if ( enable )
		sonarMode |= 1<<channel;
	else
		sonarMode &= ~(1<<channel);

	buff[0] = (char)CMD_FRAMEHEAD1;
	buff[1] = (char)CMD_FRAMEHEAD2;
	buff[2] = (char)ProtocolHeader::SONAR;
	buff[3] = (char)mode;
	buff[4] = sonarMode & 0x00FF;
	buff[5]	= (sonarMode & 0xFF00) >> 8;
	buff[6] = (char)CMD_FRAMEEND1;
	buff[7] = (char)CMD_FRAMEEND2;

	this->m_serialEngine->write((unsigned char*)buff, 8);

	m_EnabledIDStatus.push_back(ProtocolHeader::SONAR);
}

void Motion::SetPsdProc(PsdProc proc, void* pParam)
{
	m_PsdProc = proc;
	m_pPsdParam = pParam;
}

void Motion::GetPsd(UInt16  channel, bool enable)
{
	char buff[8];
	UInt16 psdMode = 0;

	memset(buff,0,sizeof(buff));
	
	if ( enable )
		psdMode |= 1<<channel;
	else
		psdMode &= ~(1<<channel);

	buff[0] = (char)CMD_FRAMEHEAD1;
	buff[1] = (char)CMD_FRAMEHEAD2;
	buff[2] = (char)ProtocolHeader::PSD;
	buff[3] = (char)0x02;
	buff[4] = psdMode & 0x00FF;
	buff[5]	= (psdMode & 0xFF00)>>8;
	buff[6] = (char)CMD_FRAMEEND1;
	buff[7] = (char)CMD_FRAMEEND2;
	this->m_serialEngine->write((unsigned char*)buff, PSD_CMD_LENGTH);

	m_EnabledIDStatus.push_back(ProtocolHeader::PSD);
}
void Motion::SetEncoderProc(EncoderProc proc, void* pParam)
{
	m_EncoderProc=proc;
	m_pEncoderParam=pParam;
}

// method for DO
void Motion::SetDO( UInt16 channel ,bool enable)
{
	char buff[8];
	UInt16 mode = 0;

	memset(buff,0,sizeof(buff));
	

	if ( enable )
		mode |= 1<<channel;
	else
		mode &= ~(1<<channel);

	buff[0] = (char)CMD_FRAMEHEAD1;
	buff[1] = (char)CMD_FRAMEHEAD2;
	buff[2] = (char)(ProtocolHeader::DO);
	buff[3] = (char)0x02;
	buff[4] = mode & 0x00FF;
	buff[5]	= (mode & 0xFF00)>>8;
	buff[6] = (char)CMD_FRAMEEND1;
	buff[7] = (char)CMD_FRAMEEND2;
	this->m_serialEngine->write((unsigned char*)buff, DO_CMD_LENGTH);
}

// method for DI
void Motion::GetDI(  UInt16 channel, bool enable )
{
	char buff[DI_CMD_LENGTH];
	UInt16 mode = 0;

	memset(buff,0,sizeof(buff));

	if ( enable )
		mode |= 1<<channel;
	else
		mode &= ~(1<<channel);

	buff[0] = (char)CMD_FRAMEHEAD1;
	buff[1] = (char)CMD_FRAMEHEAD2;
	buff[2] = (char)(ProtocolHeader::DI);
	buff[3] = (char)0x02;
	buff[4] = mode & 0x00FF;
	buff[5] = (mode & 0xFF00)>>8;
	buff[6] = (char)CMD_FRAMEEND1;
	buff[7] = (char)CMD_FRAMEEND2;
	this->m_serialEngine->write((unsigned char*)buff, DI_CMD_LENGTH);
}


void Motion::SetDA( UInt8 channel ,UInt8 value,bool enable)
{
	char buff[8];
	UInt8 mode = 0;
	if ( enable )
		mode = 1<<channel;
	else
		mode &= ~(1<<channel);
	
	buff[0] = (char)CMD_FRAMEHEAD1;
	buff[1] = (char)CMD_FRAMEHEAD2;
	buff[2] = (char)(ProtocolHeader::DA);
	buff[3] = (char)0x02;
	buff[4]	= mode;
	buff[5] = value;
	buff[6] = (char)CMD_FRAMEEND1;
	buff[7] = (char)CMD_FRAMEEND2;
	this->m_serialEngine->write((unsigned char*)buff, DA_CMD_LENGTH);
}
void Motion::SetCheckProc(SelfCheckProc proc, void* pParam)
{
	m_CheckProc = proc;
	m_pCheckParam = pParam;
}

void Motion::SelfCheck(bool enable)
{
	char buff[7];
   	 UInt8 mode;
	memset(buff,0,sizeof(buff));

	if (enable)
	{
 	       mode = (char)0x01;
	}
	else
	{
		mode = (char)0x00;
	}
	buff[0] = (char)CMD_FRAMEHEAD1;
	buff[1] = (char)CMD_FRAMEHEAD2;
	buff[2] = (char)ProtocolHeader::CHECK;
	buff[3] = (char)0x01;
	buff[4] = (char)mode;
	buff[5] = (char)CMD_FRAMEEND1;
	buff[6] = (char)CMD_FRAMEEND2;
	this->m_serialEngine->write((unsigned char*)buff, CHECK_CMD_LENGTH);	

	m_EnabledIDStatus.push_back(ProtocolHeader::CHECK);
}


