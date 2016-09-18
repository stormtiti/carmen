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
    cout << "\nThis program can read learned images and save extracted featurs into yaml file.\n"
         << "Usage: " << argv[0] << " <path to image sequence> \n"
         << "example: " << argv[0] << " /home/bob/image \n"
         << "example: " << "./save_features /home/bob/fishbird/data/hall/learning /home/bob/initialization_data/images/learning/dloop/ \n"
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
	if(argc != 3)
    {
      help(argv);
      return 1;
    }
	// Path of image sequences
	string imagePath = argv[1];
	std::string outputDir = argv[2];
	// Path of output
	//std::string outputDir = "/home/bob/initialization_data/features/dloop/";
    if (fs::is_directory(outputDir))
      fs::remove_all(outputDir);
    fs::path dir(outputDir);
    if (!fs::create_directory(dir))
    {
      cout << "ERROR -> Impossible to create the directory."<< endl;
    }

    // load image filenames
    vector<string> filenames =
    DUtils::FileFunctions::Dir(imagePath.c_str(), ".png", true);

    SurfExtractor extractor;
    vector<cv::KeyPoint> keys;
    vector<FSurf64::TDescriptor> descriptors;
    // create the learning database
    for(int i = 0; i < filenames.size(); ++i)
    {
      // get image
      cv::Mat im = cv::imread(filenames[i].c_str(), 0); // grey scale
      extractor(im, keys, descriptors);    
      // Save kp and descriptors
      char filename[80];
  	  sprintf(filename, "%05d.yml", i);
      //FileStorage fs(outputDir+lexical_cast<string>(i)+".yml", FileStorage::WRITE);
      FileStorage fs(outputDir+std::string(filename), FileStorage::WRITE);
      write(fs, "id",     i);
      write(fs, "kp",     keys);
      write(fs, "desc",   descriptors);
      fs.release();
      keys.clear();
      descriptors.clear();
      cout << "## Saving image NO: " << i <<endl;
    }

    cout<< filenames.size() <<endl;
	return 0;

}
