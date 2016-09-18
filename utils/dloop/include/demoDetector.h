/**
 * File: demoDetector.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DLoopDetector
 */

#ifndef __DEMO_DETECTOR__
#define __DEMO_DETECTOR__

#include <iostream>
#include <vector>
#include <string>

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

// DLoopDetector and DBoW2
#include "DBoW2.h"
#include "DLoopDetector.h"
#include "DUtils.h"
#include "DUtilsCV.h"
#include "DVision.h"
#include <boost/lexical_cast.hpp>
#include <boost/math/distributions.hpp>

using namespace DLoopDetector;
using namespace DBoW2;
using namespace std;
using namespace boost;
using namespace cv;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

/// Generic class to create functors to extract features
template<class TDescriptor>
class FeatureExtractor
{
public:
  /**
   * Extracts features
   * @param im image
   * @param keys keypoints extracted
   * @param descriptors descriptors extracted
   */
  virtual void operator()(const cv::Mat &im, 
    vector<cv::KeyPoint> &keys, vector<TDescriptor> &descriptors) const = 0;
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

/// @param TVocabulary vocabulary class (e.g: Surf64Vocabulary)
/// @param TDetector detector class (e.g: Surf64LoopDetector)
/// @param TDescriptor descriptor class (e.g: vector<float> for SURF)
template<class TVocabulary, class TDetector, class TDescriptor>
/// Class to run the demo 
class demoDetector
{
public:

  /**
   * @param vocfile vocabulary file to load
   * @param imagedir directory to read images from
   * @param posefile pose file
   * @param width image width
   * @param height image height
   */
  demoDetector(const std::string &vocfile, const std::string &imagedir,
    const std::string &posefile, const std::string &loopFile,int width, int height);

  demoDetector(const std::string &vocfile, const std::string &imagedir,
         int width, int height, const FeatureExtractor<TDescriptor> &extractor);

    
  ~demoDetector()
  {  
    delete m_detector;
  }

  int getLoopClosure (int, std::string, const FeatureExtractor<TDescriptor> &, cv::Mat &);

protected:

  /**
   * Reads the robot poses from a file
   * @param filename file
   * @param xs
   * @param ys
   */
  void readPoseFile(const char *filename, std::vector<double> &xs, 
    std::vector<double> &ys) const;

  void readDloopFeatures(string, vector<cv::KeyPoint>&, vector<TDescriptor>&);


protected:

  std::string m_vocfile;
  std::string m_imagedir;
  std::string m_posefile;
  int m_width;
  int m_height;
   
  DUtils::LineFile m_loopFile;
  TDetector* m_detector;
  typename TDetector::Parameters m_param;
};

// ---------------------------------------------------------------------------

template<class TVocabulary, class TDetector, class TDescriptor>
demoDetector<TVocabulary, TDetector, TDescriptor>::demoDetector
  (const std::string &vocfile, const std::string &imagedir,
  const std::string &posefile, const std::string &loopFile,int width, int height)
  : m_vocfile(vocfile), m_imagedir(imagedir), m_posefile(posefile),
    m_width(width), m_height(height)
{
	m_loopFile.OpenForAppending(loopFile.c_str());
}

template<class TVocabulary, class TDetector, class TDescriptor>
demoDetector<TVocabulary, TDetector, TDescriptor>::demoDetector
  (const std::string &vocfile, const std::string &imagedir,
  int width, int height, const FeatureExtractor<TDescriptor> &extractor)
  : m_vocfile(vocfile), m_imagedir(imagedir), m_width(width), m_height(height)
{
  //Read the vocabulary to memory
	DUtils::Profiler profiler;
    cout << "Loading vocabulary..." << endl;
    profiler.profile("load voc");
    TVocabulary voc(m_vocfile);
    profiler.stop();

  // Set loop detector parameters
    m_param.image_rows = m_height;
    m_param.image_cols = m_width;

    // Parameters given by default are:
    // use nss = true
    // alpha = 0.3
    // k = 3
    // geom checking = GEOM_DI
    // di levels = 0

    // We are going to change these values individually:
    m_param.use_nss = true; // use normalized similarity score instead of raw score
    m_param.alpha = 0.2; // nss threshold
    m_param.k = 1; // a loop must be consistent with 1 previous matches
    m_param.geom_check = GEOM_DI; // use direct index for geometrical checking
    m_param.di_levels = 2; // use two direct index levels

    // Initiate loop detector with the vocabulary
    cout << "Processing sequence..." << endl;

    m_detector = new TDetector(voc, m_param);

    // Process images
    vector<cv::KeyPoint> keys;
    vector<TDescriptor> descriptors;

    bool readFromFeatLog = true;
    vector<string> filenames;

    if (readFromFeatLog)
	{
		//std::string ymlPath = m_imagedir + "/dloop";
		std::string ymlPath = "/home/s/initialization_data/dloop"; //TODO: make it as the input entry.
		filenames = DUtils::FileFunctions::Dir(ymlPath.c_str(), ".yml", true);
	}
    else
    {
     //load image filenames
    	filenames = DUtils::FileFunctions::Dir(m_imagedir.c_str(), ".png", true);
    }

    // we can allocate memory for the expected number of images
    m_detector->allocate(filenames.size());

    std::vector<double> time;


    // create the learning database
    for(unsigned int i = 0; i < filenames.size(); ++i)
//    for(unsigned int i = 0; i < 500; ++i)
    {
        keys.clear();
        descriptors.clear();
    	// get dloop features
//    	profiler.profile("get features");
    	if (readFromFeatLog)
    	{
    		readDloopFeatures(filenames[i], keys, descriptors);
    	}
    	else
    	{
			 //get image
			cv::Mat im = cv::imread(filenames[i].c_str(), 0); // grey scale
			extractor(im, keys, descriptors);
    	}
//    	profiler.stop();

//    	profiler.profile("add database");
    	m_detector->setLearningDatabase(i, keys, descriptors);
//    	profiler.stop();

    	cout << i << endl;

//      cout << i << " size key: " << keys.size()
//    		    << " size dsp: " << descriptors.size() << endl;
    }
//    cout << endl << endl;
//    cout << "Load voc costs: " 			 << profiler.getTotalTime("load voc")     << endl;
//	cout << "Feature extraction costs: " << profiler.getTotalTime("get features")
//		 << " mean time: "     			 << profiler.getMeanTime("get features")  << endl;
//	cout << "Add database costs: " 		 << profiler.getTotalTime("add database")
//		 << " mean time: "     			 << profiler.getMeanTime("add database")  << endl;
//    cout<< filenames.size() <<endl;

/*
    cout << " Press ENTER to continue... " << endl;
    std::cin.ignore(); // Pause for debugging.
*/
}


// ---------------------------------------------------------------------------

template<class TVocabulary, class TDetector, class TDescriptor>
void demoDetector<TVocabulary, TDetector, TDescriptor>::readPoseFile
  (const char *filename, std::vector<double> &xs, std::vector<double> &ys)
  const
{
  xs.clear();
  ys.clear();
  
  fstream f(filename, ios::in);
  
  string s;
  double ts, x, y, t;
  while(!f.eof())
  {
    getline(f, s);
    if(!f.eof() && !s.empty())
    {
      //sscanf(s.c_str(), "%lf, %lf, %lf, %lf", &ts, &x, &y, &t);
      sscanf(s.c_str(), "%lf, %lf", &x, &y);
      std::cout<<"x, y = "<<x<<" "<<y<<std::endl;
      xs.push_back(x);
      ys.push_back(y);
    }
  }
  
  f.close();
}

/** \brief Get loop closure.
  * \param name the descriptor type . surf or sift 
  * \param queryID  the queried image ID
  * \param extractor   the engine for feature detection
  * \return the looped image ID
  * \author bob
  */
template<class TVocabulary, class TDetector, class TDescriptor>
int demoDetector<TVocabulary, TDetector, TDescriptor>::getLoopClosure
  (int queryID, std::string imageDir, const FeatureExtractor<TDescriptor> &extractor, cv::Mat &im_matched)
{
	// Process images
	vector<cv::KeyPoint> keys;
	vector<TDescriptor> descriptors;

	// load robot poses
	vector<double> xs, ys;

    std::string _str;
    string query_path = imageDir + "/images/infer/"+ lexical_cast<string>(queryID) + "_l.png";
    // get image
    cv::Mat im = cv::imread(query_path.c_str(), 0); // grey scale
    
    extractor(im, keys, descriptors);
    DetectionResult result;
    m_detector->detectLoop(keys, descriptors, result);


    if(result.detection())     // loop closure
    {
       cout << "### LOOP: " << result.match << " <---> " << queryID <<" ###" << endl;

       char matched_name[20];
       sprintf(matched_name, "%05d_l.png", result.match);
       string matched_path = imageDir + "/images/learning/"+ matched_name;
       string pair = lexical_cast<string>(result.match) + "-" + lexical_cast<string>(queryID);

       string matched_log_name = imageDir + "/images/loops/"+ pair + "_matched.png";
       string query_log_name = imageDir + "/images/loops/"+ pair + "_query.png";

       imwrite(query_log_name, im);
       im_matched = cv::imread(matched_path.c_str(), 0); // grey scale
       imwrite(matched_log_name, im_matched);

       /* ========== Add by Samuel for debugging =============== */
       // get image
       //cv::Mat im = cv::imread(candidate_path.c_str(), 0); // grey scale
       // show image
//       DUtilsCV::GUI::showImage(im, true, &winloop, 10);
      return result.match;
    }
    else     // not loop closure
    {
      cout << "No loop!" << endl;
    }
  
  return -1;
}

/** \brief Read dloop features from yml files.
  * \param _fileName the yaml file name 
  * \param _keys  features
  * \param _descriptors   descriptors
  * \author bob
  */
template<class TVocabulary, class TDetector, class TDescriptor>
void demoDetector<TVocabulary, TDetector, TDescriptor>::readDloopFeatures
(string fileName, vector<cv::KeyPoint>& keys, vector<TDescriptor>& descriptors)
{
    vector<TDescriptor> read_descriptors;
    FileStorage fs;
    fs.open(fileName, FileStorage::READ);
    if (!fs.isOpened())
      std::cout<< "ERROR -> Failed to open yml file." << std::endl;
    FileNode kp_node = fs["kp"];
    read(kp_node, keys);
    FileNode desc_node = fs["desc"];
    read(desc_node, read_descriptors);
    for (int j = 0; j < read_descriptors.size(); j = j + 64)
    {
        TDescriptor _vec;
        for (int k = j; k < j+64; k++)
        {
            _vec.push_back(read_descriptors[k][0]);
        }
        descriptors.push_back(_vec);
    }
    fs.release();
}


#endif

