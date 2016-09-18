
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <tf/transform_datatypes.h>
#include <libhaloc/lc.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#include <boost/filesystem.hpp>
#include "tools.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/Pose.h"
#include <fstream>
#include <iterator>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

 // DLoopDetector and DBoW2
#include "DBoW2.h" // defines Surf64Vocabulary
#include "DLoopDetector.h" // defines Surf64LoopDetector
#include "DUtilsCV.h" // defines macros CVXX 
#include "demoDetector.h"  

using namespace std;
using namespace tools;
using namespace boost;


/// This functor extracts SURF64 descriptors in the required format
class SurfExtractor: public FeatureExtractor<FSurf64::TDescriptor>
{
public:
  /** 
   * Extracts features from an image
   * @param im image
   * @param keys keypoints extracted
   * @param descriptors descriptors extracted
   */
  virtual void operator()(const cv::Mat &im, 
    vector<cv::KeyPoint> &keys, vector<vector<float> > &descriptors) const;
};
void SurfExtractor::operator() (const cv::Mat &im, 
  vector<cv::KeyPoint> &keys, vector<vector<float> > &descriptors) const
{
  // extract surfs with opencv
  static cv::SURF surf_detector(400);
  
  surf_detector.extended = 0;
  
  keys.clear(); // opencv 2.4 does not clear the vector
  vector<float> plain;
  surf_detector(im, cv::Mat(), keys, plain);
  
  // change descriptor format
  const int L = surf_detector.descriptorSize();
  descriptors.resize(plain.size() / L);

  unsigned int j = 0;
  for(unsigned int i = 0; i < plain.size(); i += L, ++j)
  {
    descriptors[j].resize(L);
    std::copy(plain.begin() + i, plain.begin() + i + L, descriptors[j].begin());
  }
}

// ----------------------------------------------------------------------------

class HacolNode
{
  public:

  HacolNode() 
  {
//    ros::NodeHandle m_nh;
    ros::NodeHandle pnh("~");
    m_tryLoops = 0;
    std::string file;
    bool infer;
    std::string base_dir;
    std::string voc_file;
    std::string img_dir;
    int img_witdh ;
    int img_height; 
    pnh.param<std::string>("model_file", file, "/home/bob/.ros/trainning_data.xml");
    pnh.param<bool>		  ("infer", infer, "false");
    pnh.param<std::string>("lc_out_path", base_dir, "/home/bob/source_code/workspace/haloc_output/");
    pnh.param<std::string>("camera_topic", m_cameraTopic, "/stereo_camera");
    pnh.param<std::string>("voc_file",    voc_file, "/home/s/source/dloop/resources");
    pnh.param<std::string>("learning_img_dir", img_dir, "/home/s/initialization_data/images/learning");
    pnh.param<int>		  ("img_witdh", img_witdh, 640);
    pnh.param<int>		  ("img_height", img_height, 480);



    m_halocLC = new haloc::LoopClosure(infer);
    m_hashTable.clear();
    m_params.work_dir 		 = base_dir; // + "work_dir";
    m_params.image_dir 		 = base_dir; // + "img_dir";
    m_params.desc_type 		 = "SURF";
    m_params.num_proj 		 = 2;
    m_params.min_neighbor 	 = 10;
    m_params.epipolar_thresh = 5;    	//1     5
    m_params.min_matches 	 = 10;       //20   5
    m_params.verbose 		 = true;
    m_params.save_images 	 = true;

    m_halocLC->setParams(m_params);
    m_halocLC->init();

    if (infer)
    {
      fs.open(file, FileStorage::READ); // infer check loop
      readData();
      setHashFormat();
    }
    else
    {
      fs.open(file, FileStorage::WRITE | FileStorage::APPEND); // training 
    }


    m_initial_pose_pub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 100);
    m_cmd_vel_pub 	   = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    m_initial_pose_sub = m_nh.subscribe("initialpose", 10, &HacolNode::handle_initialPoseCallback, this);

    image_transport::ImageTransport it(m_nh);
    image_transport::SubscriberFilter left_sub, right_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub, right_info_sub;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> image_pose;

    // Message sync
    boost::shared_ptr<Sync> 	syncInfer;
    boost::shared_ptr<SyncPose> syncLearn;

    left_sub.subscribe(it, 		 m_cameraTopic+"/left/image_rect",   10);
    right_sub.subscribe(it, 	 m_cameraTopic+"/right/image_rect",  10);
    left_info_sub.subscribe(m_nh,  m_cameraTopic+"/left/camera_info",  10);
    right_info_sub.subscribe(m_nh, m_cameraTopic+"/right/camera_info", 10);

    m_if_have_initial_pose = false;

    image_pose.subscribe(m_nh, "/amcl_pose", 10);
    syncLearn.reset(new SyncPose(SyncPolicyPose(500), left_sub, right_sub, left_info_sub, right_info_sub, image_pose) );
    syncInfer.reset(new Sync(SyncPolicy(500), left_sub, right_sub, left_info_sub, right_info_sub) );
    
    // Send the fake cmd_vel to raise particles
	geometry_msgs::Twist fakeTwist;
	fakeTwist.linear.x  = 0;
	ros::Duration(2).sleep(); // Waiting other nodes
	m_cmd_vel_pub.publish(fakeTwist);

    if (infer)
    {
      syncInfer->registerCallback(boost::bind(&HacolNode::msgsCallbackInfer, this, _1, _2, _3, _4));
      SurfExtractor extractor;
      m_dloopEngine = new demoDetector<Surf64Vocabulary, Surf64LoopDetector, FSurf64::TDescriptor> (
    		  	  	  	  	  	  	  	  	  	voc_file, img_dir, img_witdh, img_height, extractor);
    }
    else
    {
      syncLearn->registerCallback(boost::bind(&HacolNode::msgsCallbackLearn, this, _1, _2, _3, _4, _5));
    }

    ros::spin();

  }

  ~HacolNode()
  {
    fs.release();  
    delete m_dloopEngine;
  }

  void msgsCallbackLearn(
      const sensor_msgs::ImageConstPtr& l_img_msg,
      const sensor_msgs::ImageConstPtr& r_img_msg,
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg,
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
  {

    if (m_if_have_initial_pose)
    {
        cv::Mat camera_matrix_; //!> Camera matrix
        image_geometry::StereoCameraModel camera_model_; //!> Stereo camera model

        cv::Mat l_img, r_img;
        Tools::imgMsgToMat(*l_img_msg, *r_img_msg, l_img, r_img);
        
        // Get camera parameters
        Tools::getCameraModel(*l_info_msg, *r_info_msg, camera_model_, camera_matrix_);
        
        // Set camera model
        m_halocLC->setCameraModel(camera_model_, camera_matrix_);
        //m_halocLC.setParams(m_params);
        // Set image node
        if (m_halocLC->setNode(l_img, r_img) != -1)
        {
         // ROS_INFO("Set node successfully!!!");
          ImageHash curHash = m_halocLC->getCurHash();
          geometry_msgs::PoseWithCovarianceStamped curPose_ = *pose_msg;
          std::vector<float> poseVec;
          float x = curPose_.pose.pose.position.x;
          float y = curPose_.pose.pose.position.y;
          float z = curPose_.pose.pose.position.z;
          float qx = curPose_.pose.pose.orientation.x;
          float qy = curPose_.pose.pose.orientation.y;
          float qz = curPose_.pose.pose.orientation.z;
          float qw = curPose_.pose.pose.orientation.w;
          poseVec.push_back(x);
          poseVec.push_back(y);
          poseVec.push_back(z);
          poseVec.push_back(qx);
          poseVec.push_back(qy);
          poseVec.push_back(qz);
          poseVec.push_back(qw);
          saveHash(curHash, poseVec);  
        }
    }
  }


  void msgsCallbackInfer(
      const sensor_msgs::ImageConstPtr& l_img_msg,
      const sensor_msgs::ImageConstPtr& r_img_msg,
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    cv::Mat camera_matrix_; //!> Camera matrix
    image_geometry::StereoCameraModel camera_model_; //!> Stereo camera model

    cv::Mat l_img, r_img;
    Tools::imgMsgToMat(*l_img_msg, *r_img_msg, l_img, r_img);

    // Get camera parameters
    Tools::getCameraModel(*l_info_msg, *r_info_msg, camera_model_, camera_matrix_);
    
    // Set camera model
    m_halocLC->setCameraModel(camera_model_, camera_matrix_);

    // Set image node
    int query_id = m_halocLC->setNode(l_img, r_img);
    if (-1 == query_id)
    {
      ROS_ERROR("[hacol_node]: ERROR -> configure stereo image error!");
      seekLoop();
      return;
    }
    // Get loop closure 
    tf::Transform trans; 
    SurfExtractor extractor;
    string query_path = m_params.image_dir + "/images/infer/"+ boost::lexical_cast<string>(query_id) + "_l.png";

    // prepare visualization windows
    DUtilsCV::GUI::tWinHandler win = "Current image";
    DUtilsCV::GUI::showImage(l_img, true, &win, 10);
    cv::Mat matchedImage;

    // invoke dloop to detect loop closure
    int lc_found_id = m_dloopEngine->getLoopClosure(query_id, m_params.image_dir, extractor, matchedImage);

    if (lc_found_id != -1)
    {
      m_halocLC->Compute3D(lc_found_id, trans);
      std::vector<float> matched_pose = m_posTable[lc_found_id];
//      ROS_INFO("=============== LOOP CLOSED ===============")
      m_initial_pose_pub.publish(getInitialPose(matched_pose,trans));
      DUtilsCV::GUI::tWinHandler win2 = "Matched image";
      DUtilsCV::GUI::showImage(matchedImage, true, &win2, 10);
//      cout << " Press ENTER to continue... " << endl;
//      std::cin.ignore(); // Pause for debugging.

      geometry_msgs::Twist twist;
      twist.angular.z = 1.3;
      m_cmd_vel_pub.publish(twist);

      ros::Duration(15).sleep();
      twist.angular.z = 0;
      m_cmd_vel_pub.publish(twist);

      ros::shutdown(); //kill the node
    }
    else
    {
    	seekLoop();
    }
  }

   /** \brief compute the initial pose for localization with the matched pose logged in training and the
  new pose in matched frame.
  * @return The initial pose for localization.
  * \param Current pose in the frame of matched pose. 
  */
  geometry_msgs::PoseWithCovarianceStamped getInitialPose(geometry_msgs::Pose matched_pose, tf::Transform rel_motion)
  {
    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    tf::Transform matched_transform;
    tf::Quaternion quat_camera_to_base;
    quat_camera_to_base.setEuler(-M_PI/2, 0, -M_PI/2);
    tf::Transform camera_to_base(quat_camera_to_base, tf::Vector3(0, 0, 0.35));

    tf::poseMsgToTF(matched_pose, matched_transform);
    tf::Transform initial_transform = matched_transform * (camera_to_base * rel_motion * camera_to_base.inverse()); // is the inverse needed?
    tf::poseTFToMsg(initial_transform, initial_pose.pose.pose);

    std_msgs::Header header;
    header.seq      = 0;
    header.stamp    = ros::Time::now();
    header.frame_id = "map";

    initial_pose.header = header;

    // Project 3D to 2D
    initial_pose.pose.pose.position.z    = 0;
    initial_pose.pose.pose.orientation.x = 0;
    initial_pose.pose.pose.orientation.y = 0; 

    initial_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
    initial_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
    initial_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
    
    return initial_pose;
  }

    geometry_msgs::PoseWithCovarianceStamped getInitialPose(vector<float> matched_pose, tf::Transform rel_motion)
  {
    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    tf::Quaternion quat_matched_pose(matched_pose[3], matched_pose[4], matched_pose[5], matched_pose[6]);
    tf::Transform matched_transform(quat_matched_pose, tf::Vector3(matched_pose[0], matched_pose[1], matched_pose[2]));
    tf::Quaternion quat_camera_to_base;
    quat_camera_to_base.setEuler(-M_PI/2, 0, -M_PI/2);
    tf::Transform camera_to_base(quat_camera_to_base, tf::Vector3(0, 0, 0.35));

    tf::Transform initial_transform = matched_transform * (camera_to_base * rel_motion * camera_to_base.inverse()); // is the inverse needed?
    tf::poseTFToMsg(initial_transform, initial_pose.pose.pose);

    std_msgs::Header header;
    header.seq      = 0;
    header.stamp    = ros::Time::now();
    header.frame_id = "map";

    initial_pose.header = header;

    // Project 3D to 2D
    initial_pose.pose.pose.position.z    = 0;
    initial_pose.pose.pose.orientation.x = 0;
    initial_pose.pose.pose.orientation.y = 0; 

    initial_pose.pose.covariance[6*0+0] = 1.5 * 1.5;
    initial_pose.pose.covariance[6*1+1] = 1.5 * 1.5;
    initial_pose.pose.covariance[6*5+5] = M_PI/3 * M_PI/3;
    
    return initial_pose;
  }


  void handle_initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& initPose)
  {
    m_if_have_initial_pose = true;
  }

private:
  int m_tryLoops;
  ros::NodeHandle m_nh;
  demoDetector<Surf64Vocabulary, Surf64LoopDetector, FSurf64::TDescriptor>* m_dloopEngine;
  haloc::LoopClosure *m_halocLC;
  haloc::LoopClosure::Params m_params;
  ros::Publisher m_initial_pose_pub;
  ros::Publisher m_cmd_vel_pub;
  ros::Subscriber m_initial_pose_sub;
  string m_cameraTopic;
  bool m_if_have_initial_pose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo,
                                                          sensor_msgs::CameraInfo> SyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo,
                                                          sensor_msgs::CameraInfo,
                                                          geometry_msgs::PoseWithCovarianceStamped> SyncPolicyPose;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  typedef message_filters::Synchronizer<SyncPolicyPose> SyncPose;

  typedef std::pair<int, std::vector<float> > ImageHash;
  typedef std::vector<std::vector<float>> HashTableVec;
  typedef std::vector<std::pair<int, std::vector<float> > > HashTableList;

  HashTableVec m_hashTable; 
  HashTableVec m_posTable;
  FileStorage fs;

  void saveHash(ImageHash cur, std::vector<float> pos)
  {
      int id = cur.first;
      std::string hashID = "ID" + boost::lexical_cast<string>(id) ;
      std::string posID  = hashID + "-" + "pos";
      fs << hashID << cur.second;
      fs << posID << pos;
  }

  void readData()
  {
	int i = 0;
	while(true)
	{
	  std::string hashNode = "ID" + boost::lexical_cast<string>(i);
	  std::string posNode  = hashNode + "-pos";

	  if (fs[hashNode].empty() || fs[posNode].empty())
	  {
		break;
	  }
	  else
	  {
		std::vector<float> _hash;
		std::vector<float> _pos;
		_hash.clear();
		_pos.clear();
		fs[hashNode] >> _hash;
		fs[posNode]  >> _pos;

		m_hashTable.push_back(_hash);
		m_posTable.push_back(_pos);
	  }
	  i++;
	}
  }

  void setHashFormat()
  {
    HashTableList _hashTable;
    _hashTable.clear();
    for (int i = 0; i < m_hashTable.size(); i ++)
    {
      _hashTable.push_back(make_pair(i, m_hashTable[i]));
    }
    m_halocLC->setHashData(_hashTable);
  }

  void seekLoop()
  {
      geometry_msgs::Twist twist;
      twist.angular.z = 1.3;
      m_cmd_vel_pub.publish(twist);

      ros::Duration(0.2).sleep();
      twist.angular.z = 0;
      m_cmd_vel_pub.publish(twist);
      m_tryLoops++;
      cout << m_tryLoops << "# try." << endl;
  }
};


int main (int argc, char** argv)
{
  ros::init(argc, argv, "libhaloc_lc");
  HacolNode hacolnode;

  // ROS spin
  ros::Rate r(50);
  while (ros::ok())
  {
    r.sleep();
  } 
  return 0;
}
