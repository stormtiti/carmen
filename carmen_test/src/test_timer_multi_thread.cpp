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

		m_spinner = new ros::MultiThreadedSpinner(4);

		runTimer12();
		runTimer34();

		//ros::spin();
	}

	~TimerTester()
	{
	}

	void runTimer12( )
	{
		ros::Timer timer1 = m_n.createTimer(ros::Duration(0.1),
				&TimerTester::callback1, this, true);
		ros::Timer timer2 = m_n.createTimer(ros::Duration(3),
				&TimerTester::callback2, this, true);
//		m_spinner->spin();
		ros::spinOnce();
	}

	void runTimer34( )
	{
		ros::Timer timer3 = m_n.createTimer(ros::Duration(0.5),
				&TimerTester::callback3, this, true);
		ros::Timer timer4 = m_n.createTimer(ros::Duration(3),
				&TimerTester::callback4, this, true);
//		m_spinner->spin();
		ros::spinOnce();
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
		twist.linear.x  = 2;
		twist.linear.y  = 2;
		twist.linear.z  = 2;

		twist.angular.x = 2;
		twist.angular.y = 2;
		twist.angular.z = 2;
		m_cmdVelPub.publish(twist);
	}

	void callback3(const ros::TimerEvent&)
	{
		ROS_INFO("Callback 3 triggered");
		geometry_msgs::Twist twist;
		twist.linear.x  = 3;
		twist.linear.y  = 3;
		twist.linear.z  = 3;

		twist.angular.x = 3;
		twist.angular.y = 3;
		twist.angular.z = 3;
		m_cmdVelPub.publish(twist);

	}

	void callback4(const ros::TimerEvent&)
	{
		ROS_INFO("Callback 4 triggered");
		geometry_msgs::Twist twist;
		twist.linear.x  = 4;
		twist.linear.y  = 4;
		twist.linear.z  = 4;

		twist.angular.x = 4;
		twist.angular.y = 4;
		twist.angular.z = 4;
		m_cmdVelPub.publish(twist);
	}

private:
	ros::Publisher m_cmdVelPub;
	ros::NodeHandle m_n;
	ros::MultiThreadedSpinner *m_spinner;

};
 
 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "test_timer_multi_thread");
//   ros::NodeHandle n;
   ROS_INFO("Node started");
   TimerTester iTimer;
//   iTimer.run();
 //  ros::spin();
   ROS_INFO("Node stopped");
   return 0;
}
