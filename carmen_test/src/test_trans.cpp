//#include <ros/ros.h>
//#include <iostream>
//#include <libhaloc/lc.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
//
//using namespace std;
//
//
//int main (int argc, char** argv)
//{
//// /home/s/Pictures/160728loop/5/620-1239_matched.png
////	-4.46827507e+00 4.13735723e+00 0. 0. 0. -5.16597867e-01 8.56228173e-01
//// /home/s/Pictures/160728loop/5/620-1239_query.png
//	ros::init(argc, argv, "libhaloc_lc");
//	ros::NodeHandle n;
//	ros::Publisher p = n.advertise<geometry_msgs::PoseWithCovarianceStamped>
//								("initialpose", 100);
//	float pose[] = {-4.46827507e+00, 4.13735723e+00, 0, 0, 0, -5.16597867e-01, 8.56228173e-01};
//	vector<float> matched_pose (pose, pose + sizeof(pose) / sizeof(float) );
//
//	geometry_msgs::PoseWithCovarianceStamped initial_pose;
//	tf::Quaternion quat_matched_pose(matched_pose[3], matched_pose[4], matched_pose[5], matched_pose[6]);
//	tf::Transform matched_transform(quat_matched_pose, tf::Vector3(matched_pose[0], matched_pose[1], matched_pose[2]));
//	tf::Quaternion quat_camera_to_base;
//	quat_camera_to_base.setEuler(-M_PI/2, 0, -M_PI/2);
//	tf::Transform camera_to_base(quat_camera_to_base, tf::Vector3(0, 0, 0.35));
//
//	tf::Transform initial_transform = matched_transform * (camera_to_base * rel_motion * camera_to_base.inverse()); // is the inverse needed?
//	tf::poseTFToMsg(initial_transform, initial_pose.pose.pose);
//
//	std_msgs::Header header;
//	header.seq      = 0;
//	header.stamp    = ros::Time::now();
//	header.frame_id = "map";
//
//	initial_pose.header = header;
//
//	// Project 3D to 2D
//	initial_pose.pose.pose.position.z    = 0;
//	initial_pose.pose.pose.orientation.x = 0;
//	initial_pose.pose.pose.orientation.y = 0;
//
//	initial_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
//	initial_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
//	initial_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
//
//	p.publish(initial_pose);
//
//	ros::spin();
//}
