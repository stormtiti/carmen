#include "ros/ros.h" 
class TimerTester
{
public:
    TimerTester()
    {
        ros::NodeHandle n;
        ros::Timer timer1 = n.createTimer(ros::Duration(0.001),
                &TimerTester::callback1, this, false);
        ros::Timer timer2 = n.createTimer(ros::Duration(2),
                &TimerTester::callback2, this, false);
        ros::spin();
    }
    ~TimerTester()
    {
    }
    void callback1(const ros::TimerEvent&)
    {
        ROS_INFO("Callback 1 triggered");
    }
    void callback2(const ros::TimerEvent&)
    {
        ROS_INFO("Callback 2 triggered");
    }
};
 
 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "test_timer");
   TimerTester iTimer;
   return 0;
}
