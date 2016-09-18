#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"


class TimerTester
{
public:
	TimerTester()
	{
//		ros::NodeHandle m_n;
		m_cmdVelPub  = m_n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
		geometry_msgs::Twist twist;
		twist.linear.x  = 8;
		twist.linear.y  = 8;
		twist.linear.z  = 8;

		twist.angular.x = 8;
		twist.angular.y = 8;
		twist.angular.z = 8;

		ros::Duration(0.1).sleep();//A small wait was necessary for the publishing and subscribing to begin working

		m_cmdVelPub.publish(twist);

		twist.linear.x  = 7;
		twist.linear.y  = 7;
		twist.linear.z  = 7;

		twist.angular.x = 7;
		twist.angular.y = 7;
		twist.angular.z = 7;
		m_cmdVelPub.publish(twist);
		ROS_INFO("Publishing...");
		//ros::spin();
	}

	~TimerTester()
	{
	}

	void run( )
	{
		ros::Timer timer1 = m_n.createTimer(ros::Duration(0.1),
				&TimerTester::callback1, this, true);
		ros::Timer timer2 = m_n.createTimer(ros::Duration(3),
				&TimerTester::callback2, this, true);
		ros::spin(); // It must be the only one spin in system, otherwise the timer can't work!
	}

	void callback1(const ros::TimerEvent&)
	{
		ROS_INFO("Callback 1 triggered");
		geometry_msgs::Twist twist;
		twist.linear.x  = 1;
		twist.linear.y  = 1;
		twist.linear.z  = 1;

		twist.angular.x = 1;
		twist.angular.y = 1;
		twist.angular.z = 1;
		m_cmdVelPub.publish(twist);

	}

	void callback2(const ros::TimerEvent&)
	{
		ROS_INFO("Callback 2 triggered");
		geometry_msgs::Twist twist;
		twist.linear.x  = 0;
		twist.linear.y  = 0;
		twist.linear.z  = 0;

		twist.angular.x = 0;
		twist.angular.y = 0;
		twist.angular.z = 0;
		m_cmdVelPub.publish(twist);
	}

private:
	ros::Publisher m_cmdVelPub;
	ros::NodeHandle m_n;

};
 
 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "test_timer_vel_in_member_function");
//   ros::NodeHandle n;
   ROS_INFO("Node started");
   TimerTester iTimer;
   iTimer.run();
 //  ros::spin();
   ROS_INFO("Node stopped");
   return 0;
}
