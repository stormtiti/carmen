#include "Motion.h"
#include <cstdio>
#include <iostream>
#include <fstream>
//#include <cmath>
#include "math.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"          // odometry message
#include <tf/transform_broadcaster.h>   // tf broadcaser 
#include <geometry_msgs/Twist.h>        // velocity command message
#include <geometry_msgs/PoseStamped.h>        // velocity command message



const Float32 B = 0.401;                 // base line distance (m)

//std::ofstream EncoderLogFile("/home/mpig/abs_encoder.csv", std::ios::trunc);

using namespace carmen;

class CarmenNode
{

	public:
		CarmenNode() 
		{
			X = 0;
			Y = 0; 
			TH = 0;
			m_mode = 0;
		}
		~CarmenNode()
		{

		}

		// configure motion control callback
  		void ConfigureMotionCallback()
  		{
  			void* param;
  			m_MotionDriver.SetEncoderAbsProc(boost::bind(&CarmenNode::handle_MotionCallback, this,_1,_2,_3,_4,_5,_6), param);

  			Float32 lv,rv,ls,rs;
  			lv = -1.0;
  			rv = 1.0;
  			ls = 2.0;
  			rs = 2.0;
  			//handle_MotionCallback(param, &lv,&rv,&ls,&rs,0.5);
  		}

		/*
		*  Get encoder abstract value for wrapping them into ROS odometry message
		*  (lv rv) -------------> (nav_msgs::Odometry) 
		*  @param[in] lv  Left velocity
		*  @param[in] rv  Right velocity
		*  @param[in] ls  Left distance  (not used)
		*  @param[in] rs  Right distance  (not used)
		*  @param[in] time Sample period  (s)
		*/ 
		 void handle_MotionCallback(void* par, Float32* lv, Float32* rv, Float32* ls, Float32* rs, Float32 t)
		{
			Float32 vx = 0;
			Float32 vy = 0;
			Float32 vth = 0;
			
		/*	std::cout<<">>>>>> Output encoder absolute value >>>>>>>>>>"<<std::endl;
	    std::cout<<"-------left motor: v and s "<<*lv<<","<<*ls<<std::endl;
	    std::cout<<"-------right motor v and s: "<<*rv<<","<<*rs<<std::endl;
	    std::cout<<"-------Time stamp: "<<t<<std::endl; */
			
			SpeedToOdometry(*lv, *rv, vx, vy, vth, *ls, *rs, t);
			FillRosOdoMSG(vx, vy, vth);	
			FillRosOdoPoseMSG();
		}

		void handle_SonarCallback()
		{}
		void handle_PsdCallback()
		{}

		// ROS subscriber for top vel_
		void handle_cmdVelCallback(const geometry_msgs::Twist& cmd_vel)
		{
			ROS_INFO("Received a /cmd_vel message!");  
    		ROS_INFO("Linear Components:[%f,%f,%f]",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);  
   			ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);  

   			Float32 lv, rv; // left and right velocity

   			 CmdVelToSpeed(cmd_vel, lv, rv);
   			// todo: send command to carmen robot
   			 MoveCarmen(lv, rv);
		}

		// get parameters from ROS server
		void GetROSParameters() 
		{
    		m_nhROS.param<std::string>("port", m_port, "/dev/ttyS1");
    		m_nhROS.param<int>("baudrate", m_baudrate, 19200);
    	
    		m_nhROS.param<int>("move_mode", m_moveMode, 1); 
    		m_nhROS.param<int>("pid_mode", m_pidMode, 1);
    		m_nhROS.param<float>("velocity/left", m_leftVel, 0.1);
    		m_nhROS.param<float>("velocity/right", m_rightVel, 0.1);
    		m_nhROS.param<float>("distance/left", m_leftDis, 1);
    		m_nhROS.param<float>("distance/right", m_rightDis, 1);
    		m_nhROS.param<float>("acc/left", m_leftAcc, 0.3);
    		m_nhROS.param<float>("acc/right", m_rightAcc, 0.3);
    		m_nhROS.param<int>("encoder/mode", m_encoderMode, 0);
    		m_nhROS.param<bool>("encoder/enable", m_enableFlag, 1);
    		m_nhROS.param<int>("pid/p", m_p, 0);
    		m_nhROS.param<int>("pid/i", m_i, 0);
    		m_nhROS.param<int>("pid/d", m_d, 0); 

  		}

  		// configure ROS publisher and subscriber
  		void ConfigureROSPublisher() 
  		{
    		m_pubEncoder  = m_nhROS.advertise<nav_msgs::Odometry>("odom", 1000);
    		m_pubVelCMD   = m_nhROS.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    		m_pubOdomPose = m_nhROS.advertise<geometry_msgs::PoseStamped>("pose_odom", 1);
    		m_subVelCMD   = m_nhROS.subscribe("/cmd_vel", 1000, &CarmenNode::handle_cmdVelCallback, this);
    	//	ROS_INFO("Start publisher and subscriber!!!");
  		}

  		

  		/*
  		*	Setup the node. 
  		*	The following process will be done:
  		*   (1) Configure serial and connect the serial port
  		*	(2) Start reading thread
  		*/
  		void Setup()
  		{
  			SERIAL_INFO serialDsc;
  			serialDsc.port = m_port;
  			serialDsc.baudrate = m_baudrate;
  			m_MotionDriver.Init(serialDsc);

  			if (m_MotionDriver.connect())
  			{
  				ROS_INFO("Connect successfully!!!");
  				m_MotionDriver.GetMotion(m_encoderMode,m_enableFlag);
				m_MotionDriver.KickoffReading();
  			}
  		}

 

	private:
		// Core motion control engine class 
		Motion m_MotionDriver;
		ros::Time timeStamp;
		ros::NodeHandle m_nhROS;
		std::string m_port;
		UInt16 m_mode; // mode for getting encoder value or not : 1: encoder
		int m_baudrate;
 	 	ros::Publisher m_pubEncoder;
 	 	ros::Publisher m_pubOdomPose; // Publisher for pose based on raw odometry
 	 	ros::Publisher m_pubSonar;
 	 	ros::Publisher m_pubPSD;
 	 	ros::Publisher m_pubVelCMD;
 	 	ros::Subscriber m_subVelCMD;
 	 	tf::TransformBroadcaster m_odomBroadcaster;


 	 	nav_msgs::Odometry m_OdometryMsg;

 	 	geometry_msgs::TransformStamped m_OdomTransMsg;
 	 	geometry_msgs::PoseStamped m_OdomPoseMsg;
		Float32 X, Y, TH;   // position and orientation
		int m_moveMode;
		int m_pidMode;
		Float32 m_leftVel;
		Float32 m_rightVel;
		Float32 m_leftDis;
		Float32 m_rightDis;
		Float32 m_leftAcc;
		Float32 m_rightAcc;
		int m_encoderMode;
		bool m_enableFlag;
		int m_p, m_i, m_d;

  	private:

  		/*
  		*	Send command to control carmen 
  		*/
  		void MoveCarmen(Float32 lv, Float32 rv)
  		{
  			m_MotionDriver.Drive(m_moveMode, m_pidMode, 
  								lv, m_p, m_i, m_d, m_leftAcc, m_leftDis,
  								rv, m_p, m_i, m_d, m_rightAcc, m_rightDis);
  		}  

  		/*
  		* Convert ROS cmd_vel topic to left and right velocity for send command to robot
  		*
  		*  (geometry_msgs::Twist) -------> (lv rv) 
  		*	@param[in] cmd_vel Velocity command ROS message 
  		*	@param[out] lv  Left velocity of motor
  		*	@param[out] rv  Right velocity of motor
  		*/
  		void CmdVelToSpeed(const geometry_msgs::Twist& cmd_vel, Float32& lv,Float32& rv)
  		{
  			Float32 cent_speed = cmd_vel.linear.x ;
  			lv = cent_speed - (cmd_vel.angular.z) * B / 2;
  			rv = cent_speed + (cmd_vel.angular.z) * B / 2;
  			lv = lv * 1.02;

  			//ROS_INFO("wheel left velocity: %f   ", lv);
  			//ROS_INFO("wheel right velocity: %f  ", rv);


  		}  

  		/*!
  		*	Convert left and right velocity to robot odometry 
  		*   (lv rv) ----------> (x y th vx vy)
  		*   @param[in] lv Left velocity   (m/s)
  		*   @param[in] rv Right velocity
  		*   @param[out] x  Translation in x cordinate 
  		*	@param[out] y  Translation in y cordinate
  		*	@param[out] th  Angular in z cordinate  ( rad)
  		*	@param[out] vx  Velocity in x cordinate (m/s)
  		*	@param[out] vy  Velocity in y cordinate  (m/s)
  		*   @param[out] vth  Angular velocity     (rad/s)
  		*/
  		void SpeedToOdometry(Float32 lv, Float32 rv, Float32& vx, Float32& vy, Float32& vth, Float32 ls, Float32 rs, Float32 t)
  		{
  			Float32 cenv = (lv + rv) / 2;
  			Float32 delta_speed = rv - lv;

  			vth = delta_speed / B;      // angular velocity
  			TH = TH + vth * t;      // angular
        // todo: add angular constrain

  			vx = cenv * cos(TH);       // velocity in x axis
  			vy = cenv * sin(TH);       // velocity in y axis
  			 
  			X = X + vx * t;             // distance displacement in x axis
  			Y = Y + vy * t;             // distance displacement in y axis  

       // EncoderLogFile << lv <<"," << rv << "," << ls << "," << rs << "," << t << "," << vx << "," << vy << ","<< vth <<"\n";			
  		} 

  		/*!
  		*	Fill nav_msgs::Odometry using data from member function [speedToOdometry]
  		*   Result: fill member variable m_OdometryMsg
  		*   (x y th vx vy) ----------> (nav_msgs::Odometry)
  		*   @param[in] x  Translation in x cordinate 
  		*	@param[in] y  Translation in y cordinate
  		*	@param[in] th  Angular in z cordinate  ( rad)
  		*	@param[in] vx  Velocity in x cordinate (m/s)
  		*	@param[in] vy  Velocity in y cordinate  (m/s)
  		*/
  		void FillRosOdoMSG(Float32 vx_, Float32 vy_, Float32 vth_)
  		{
  			// First:  publish odometry topic
  			timeStamp = ros::Time::now();
			m_OdometryMsg.header.stamp = timeStamp;
			m_OdometryMsg.header.frame_id = "odom";
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(TH);

			  //set the position
  			m_OdometryMsg.pose.pose.position.x = X;
  			m_OdometryMsg.pose.pose.position.y = Y;
  			m_OdometryMsg.pose.pose.position.z = 0.0;
  			m_OdometryMsg.pose.pose.orientation = odom_quat;
 
  		    //set the velocity
  			m_OdometryMsg.child_frame_id = "base_link";
  			m_OdometryMsg.twist.twist.linear.x = vx_;
  			m_OdometryMsg.twist.twist.linear.y = vy_;
  			m_OdometryMsg.twist.twist.angular.z = vth_;
  			//publish the message
  			m_pubEncoder.publish(m_OdometryMsg);

  			//ROS_INFO("Publish topic successfully!!!");

			// Second: publish the transform over tf
			m_OdomTransMsg.header.stamp = timeStamp;
			m_OdomTransMsg.header.frame_id = "odom";
			m_OdomTransMsg.child_frame_id = "base_link";
			m_OdomTransMsg.transform.translation.x = X;
			m_OdomTransMsg.transform.translation.y = Y;
			m_OdomTransMsg.transform.translation.z = 0.0;
			m_OdomTransMsg.transform.rotation = odom_quat;
			m_odomBroadcaster.sendTransform(m_OdomTransMsg);

			//ROS_INFO("Send transformation successfully!!!");
  		}


      void FillRosOdoPoseMSG( )
      {
        // First:  publish odometry topic
        m_OdomPoseMsg.header.stamp = timeStamp;
        m_OdomPoseMsg.header.frame_id = "map";
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(TH);

        //set the position
        m_OdomPoseMsg.pose.position.x = X;
        m_OdomPoseMsg.pose.position.y = Y;
        m_OdomPoseMsg.pose.position.z = 0.0;
        m_OdomPoseMsg.pose.orientation = odom_quat;
 
        m_pubOdomPose.publish(m_OdomPoseMsg);

        //ROS_INFO("Publish pose based on odometry successfully!!!");
      }

};

int main (int argc, char** argv) 
{
	ros::init(argc, argv, "carmen_node");
	CarmenNode CarmenDriver;
	CarmenDriver.GetROSParameters();
	CarmenDriver.ConfigureROSPublisher();
	CarmenDriver.ConfigureMotionCallback();
	CarmenDriver.Setup();
//	ros::Rate loop_rate(10);

//	while (ros::ok())
		
	//	CarmenDriver.ConfigureMotionCallback();
//				loop_rate.sleep();
  //	ros::spinOnce();

	// CarmenDriver.Setup();
	ros::spin();
  //EncoderLogFile.close();
	return 0;

}



