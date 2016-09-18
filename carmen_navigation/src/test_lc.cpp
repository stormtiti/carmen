
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

using namespace std;

void help(char** argv)
{
    cout << "\nThis program can try haloc LoopClosure library through reading a sequence of stereo images.\n"
         << "Usage: " << argv[0] << " <path to stereo sequence>" << " <image format>\n"
         << "example: " << argv[0] << "/home/bob/image " << " right%%02d.jpg\n"
         << "q,Q,esc -- quit\n"
         << endl;
}
int main (int argc, char** argv)
{
	if(argc != 3)
    {
      help(argv);
      return 1;
    }
	// Read image sequences
	string basePath = argv[1];
	string imageFormat = argv[2];
	string leftImagePath = basePath + "/left/" + imageFormat;
	string rightImagePath = basePath + "/right/" + imageFormat;
    cv::VideoCapture leftSeq(leftImagePath);
    cv::VideoCapture rightSeq(rightImagePath);
    if (!leftSeq.isOpened() || !leftSeq.isOpened())
    {
      cerr << "Failed to open stereo image sequence!\n" << endl;
      return 1;
    }
    
    // Create haloc object
   	haloc::LoopClosure halocEngine(false);
	haloc::LoopClosure::Params params;
	std::string base_str = "/home/bob/source_code/workspace/haloc_output/";
	params.work_dir = base_str + "work_dir";
	params.image_dir = base_str + "image_dir";
	params.num_proj = 3;
	params.min_neighbor = 2;
	halocEngine.setParams(params);
	halocEngine.init();

    // Main loop
    cv::Mat leftImg, rightImg;
    int loop_closure_with; 	// <- Will contain the index of the image that closes loop with the last inserted (-1 if none).
	tf::Transform trans; 	// <- Will contain the transformation of the loop closure (if any).
	int index = 0;
    for(;;)
    {
      leftSeq >> leftImg;
      rightSeq >> rightImg;

      if(leftImg.empty() || rightImg.empty())
      {
          cout << "End of Sequence" << endl;
          break;
      }
     // imshow("left | q or esc to quit", leftImg);
      if (halocEngine.setNode(leftImg))
      {
      	std::cout<<"Set node successfully!!!"<<std::endl;
      }
      else
      {
      	 std::cout<<"can't compute 3d points"<<std::endl;
      }
      if (halocEngine.getLoopClosure(loop_closure_with, trans))
      {
      		std::cout << "[libhaloc:] LC by hash between nodes " <<
                      ++index << " and " << loop_closure_with << std::endl;
      }
      else
      {
      	std::cout<<"Failed to get loop closure!"<<std::endl;
      }
    }

	return 0;

}
