#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <boost/lexical_cast.hpp>
#include "DBoW2.h"

#include "DLoopDetector.h"
#include "DUtils.h"
#include "DUtilsCV.h"
#include "DVision.h"
#include "demoDetector.h"

#include <boost/filesystem.hpp>

using namespace std;
using namespace DLoopDetector;
using namespace DBoW2;
using namespace cv;

using namespace boost;
namespace fs = filesystem;

void help(char** argv)
{
    cout << "\nThis program can read extracted featurs from yaml file.\n"
         << "Usage: " << argv[0] << " <path to yml> \n"
         << "example: " << argv[0] << " /home/bob/image \n"
         << "example: " << "./read_features /home/bob/initialization_data/features/dloop/ \n"
         << endl;
}

// ----------------------------------------------------------------------------
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

int main (int argc, char** argv)
{
	if(argc != 2)
    {
      help(argv);
      return 1;
    }
     DUtils::Profiler profiler;
	// Path of image sequences
	string ymlPath = argv[1];
    // load image filenames
    vector<string> filenames =
    DUtils::FileFunctions::Dir(ymlPath.c_str(), ".yml", true);
    profiler.profile("read features");
    // create the learning database
    for(int i = 0; i < filenames.size(); ++i)
    {
      // get image
      vector<cv::KeyPoint> keys;
      vector<FSurf64::TDescriptor> read_descriptors;
      vector<FSurf64::TDescriptor> final_descriptors;
      // Get the image keypoints and descriptors
    FileStorage fs;
    fs.open(filenames[i], FileStorage::READ);
    if (!fs.isOpened())
      std::cout<< "ERROR -> Failed to open yml file." << std::endl;
    int id;
    fs["id"] >> id;
    FileNode kp_node = fs["kp"];
    read(kp_node, keys);
    FileNode desc_node = fs["desc"];
    read(desc_node, read_descriptors);
    for (int j = 0; j < read_descriptors.size(); j = j + 64)
    {
        FSurf64::TDescriptor _vec;
        for (int k = j; k < j+64; k++)
        {
            _vec.push_back(read_descriptors[k][0]);
        }
        final_descriptors.push_back(_vec);
    }
    fs.release();
     // cout << "## Reading image NO: " << i <<endl;
    }
    profiler.stop();
    cout << endl << "Execution time:" << profiler.getMeanTime("read features") << endl;

    cout<< filenames.size() <<endl;
	return 0;

}
