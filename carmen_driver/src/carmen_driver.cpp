#include <iostream>
#include "carmen_driver.h"

using namespace carmen;

inline void printHex(char *data, int length)
{
  for (int i = 0; i < length; ++i) {
    printf("0x%.2X ", (unsigned)(unsigned char)data[i]);
  }
  printf("\n");
}

inline void printHexFromString(std::string str)
{
  printHex(const_cast<char *>(str.c_str()), str.length());
}

/////////////////////////////////////////////////////////////////////////////

CarmenDriver::CarmenDriver() : m_configured(false), m_baudrate(19200), m_port("/dev/ttyS1") 
{
  this->m_connected = false;
}

CarmenDriver::~CarmenDriver()
{
  this->disconnect();
}

void CarmenDriver::configure(std::string port, int baudrate)
{
  this->m_port = port;
  this->m_baudrate = baudrate;
  this->m_configured = true;
}

void CarmenDriver::connect()
 {
  if(!this->m_configured) 
  {
    CARMEN_THROW_MSG(ConnectionFailedException, "The serial port must be "
      "configured before connecting.");
  }
  try 
  {
      // Configure and open the serial port
      this->m_serial.setPort(this->m_port);
      this->m_serial.setBaudrate(this->m_baudrate);
      serial::Timeout timeout = serial::Timeout::simpleTimeout(200);
      this->m_serial.setTimeout(timeout);
      this->m_serial.open();
  } catch(std::exception &e) 
  {
      CARMEN_THROW_MSG(ConnectionFailedException, e.what());
  }
  this->m_connected = true;
}

void CarmenDriver::disconnect() 
{
  if(this->m_connected) 
  {
      if(this->m_serial.isOpen())
      {
        this->m_serial.close();    
      }
          
      this->m_connected = false;
  }
}

int CarmenDriver::read(unsigned char* buffer, int size) 
{
  return this->m_serial.read(buffer, size);
}

int CarmenDriver::write(unsigned char* buffer, int size) 
{
  return this->m_serial.write(buffer, size);
}


bool CarmenDriver::BuildPacket(Packet &packet) {
  if(!this->m_connected)
    CARMEN_THROW_MSG_AND_ID(PacketRetrievalException, "Not connected.", 1);
  
  bool packet_complete = false;
  int packet_index = 0;

  if (packet.size() != 0)
  {
    packet.clear();
  }
  
  while(!packet_complete && !this->m_canceled) {
    // Top the buffer off
    int prev_size = this->m_dataBuffer.size();
    if(prev_size < MINIMIZE_BUFFER_NUM) {
      this->fillBuffer();
      // Ensure that data was read into the buffer
      if(prev_size == this->m_dataBuffer.size()) {
        CARMEN_THROW_MSG_AND_ID(PacketRetrievalException, "No data received "
          "from Carmen.", 3);
        return false;
      }
    }
    // Looking for the first byte header of packet 0x5A
    if(packet_index == 0 && this->m_dataBuffer[0] == RES_FRAMEHEAD1) 
    {
      // Put the first char head in the packet
      packet.push_back(this->m_dataBuffer[0]);
      // Remove the 0xF0 from the buffer
      this->m_dataBuffer.erase(this->m_dataBuffer.begin());
      // Look for next packet byte
      packet_index += 1;
    } 
    else if (packet_index == 0) 
    { 
      // If we were looking for the first byte, but didn't find it
      // Remove the invalid byte from the buffer
      this->m_dataBuffer.erase(this->m_dataBuffer.begin());
    }
    
    // Looking for second byte of packet 0xA5
    if(packet_index == 1 && this->m_dataBuffer[0] == RES_FRAMEHEAD2) 
    {
      packet.push_back(this->m_dataBuffer[0]);
      // Remove the 0x55 from the buffer
      this->m_dataBuffer.erase(this->m_dataBuffer.begin());
      // Look for next packet byte
      packet_index += 1;
    } 
    else if (packet_index == 1) 
    { // Else were looking for second byte but didn't find it
      // Reset the packet index to start search for packet over
      packet_index = 0;
    }
    
    // Looking for more bytes 
    if(packet_index >= 2) 
    {
      // Put the next byte in the packet
      packet.push_back(this->m_dataBuffer[0]);
      // if 
      if ((this->m_dataBuffer[0] == RES_FRAMEEND1) && (this->m_dataBuffer[1] == RES_FRAMEEND2))
      {
        packet.push_back(this->m_dataBuffer[1]);
        // the packe is completed
        packet_complete = true;
        //remove two byte which represent frame end ( 0xB5 0x5B)
        this->m_dataBuffer.erase(this->m_dataBuffer.begin(),this->m_dataBuffer.begin() + 1);
      }
      else
      {
        // Remove the byte from the buffer
        this->m_dataBuffer.erase(this->m_dataBuffer.begin());
      }
      
      // Look for next packet byte
      packet_index += 1;
    }
    
  }
  return true;
}

void CarmenDriver::fillBuffer() {
  unsigned char buffer[BUFFER_SIZE];
  // Read up to BUFFER_SIZE what ever is needed to fill the vector
  // to BUFFER_SIZE
  int bytes_read = this->read(buffer, BUFFER_SIZE - m_dataBuffer.size());
//	printHex((char*)buffer,bytes_read);	
  // Append the buffered data to the vector
  this->m_dataBuffer.insert(this->m_dataBuffer.end(), buffer, buffer+bytes_read);
}
