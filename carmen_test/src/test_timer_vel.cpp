#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"


class TimerTester
{
public:
	TimerTester()
	{
		ros::NodeHandle n;
		m_cmdVelPub  = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
		ros::Timer timer1 = n.createTimer(ros::Duration(0.1),
				&TimerTester::callback1, this, true);
		ros::Timer timer2 = n.createTimer(ros::Duration(0.2),
				&TimerTester::callback2, this, true);
		ros::spin();
	}

	~TimerTester()
	{
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

};
 
 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "test_timer_vel");
   TimerTester iTimer;
   return 0;
}
