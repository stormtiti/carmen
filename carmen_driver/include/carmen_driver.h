/*!
 * \file carmen_driver.h
 * \author  bob <b@mpig.com.cn>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The BSD License
 *
 *
 * \section DESCRIPTION
 *
 * This provides a Serial based implementation for carmen robot.
 * 
 * The implementation is based on William Woodall's segwayRMP ROS project:
 * https://github.com/zhangaigh/libsegwayrmp
 *
 */

#ifndef _CARMEN_DRIVER_H_
#define _CARMEN_DRIVER_H_

#include "GlobalDef.h"
#include "serial/serial.h"

namespace carmen 
{

        typedef std::vector<unsigned char> Packet;

/*!
 * Provides a driver class for reading and writing packets through serial port.
 */
class CarmenDriver {
public:


    /*!
    * A structure to represent the data packet  
    */



    /*!
     * Constructs the CarmenDriver object.
     */
    CarmenDriver();
    ~CarmenDriver();
    
    /*!
     * Connects to the serial port if it has been configured. Can throw ConnectionFailedException.
     */
    void connect();
    
    /*!
     * Disconnects from the serial port if it is open.
     */
    void disconnect();
    
    /*!
     * Read Function, reads from the serial port.
     * 
     * \param buffer An unsigned char array for data to be read into.
     * \param size The amount of data to be read.
     * \return int Bytes read.
     */
    int read(unsigned char* buffer, int size);
    
    /*!
     * Write Function, writes to the serial port.
     * 
     * \param buffer An unsigned char array of data to be written.
     * \param size The amount of data to be written.
     * \return int Bytes written.
     */
    int write(unsigned char* buffer, int size);
    
    /*!
     * Configures the serial port.
     * 
     * \param port The com port identifier like '/dev/ttyS1' on UNIX and like 'COM1' on windows.
     * \param baudrate The speed of the serial communication.
     */
    void configure(std::string port, int baudrate);

    /*!
    * A function to see if the serial port is connected.
    * 
    * \return bool weather or not the port is connected.
    */
    bool isConnected() {return this->m_connected;}

    /*!
   * This function reads from the serial port and returns one complete packet.
   * 
   * \param packet A packet by reference to be read into.
   */

    bool BuildPacket(Packet& packet);

    /*!
    *   This fucntion is used to parse the packet and return the value according to user protocal. 
    *   Here, a callback function is invoked for user's operation. In ROS, topic or service can be defined for other node.
    */
    bool ParsePacket(const Packet& packet);

    /*!
    * Cancels any currently being processed packets, should be called at shudown.
    */
    void cancel() {this->m_canceled = true;}

private:

    
    void fillBuffer();
    
private:

    bool m_connected;    
    bool m_canceled;
    bool m_configured;
  
    std::vector<unsigned char> m_dataBuffer;   /**< the data buffer*/
    bool _mconfigured;
    
    std::string m_port;
    int m_baudrate;
    
    serial::Serial m_serial;                 /**< the serial engine*/
};

}

#endif