/*
 *  The tool is used to edit the map 
 *
 *  Created on:  2016.08.31
 *      Author: bob
 */
#include "edit_map_tool.h"
#include "rviz/ogre_helpers/line.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/load_resource.h"

#include <OgreSceneNode.h>

#include <sstream>
 #include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <string.h>

 #include "math.h"

namespace rviz
{

  #ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

EditMapTool::EditMapTool() :
    state_(START),
    length_(-1)
{
  setIcon( loadPixmap("package://rviz/icons/obstacle.jpg") );
}

EditMapTool::~EditMapTool()
{
  delete line_;
}

void EditMapTool::onInitialize()
{
  line_ = new Line(context_->getSceneManager());
  line_->setColor(Ogre::ColourValue::Blue);
  setName( "Edit map" );

  std_cursor_ = getDefaultCursor();
  //hit_cursor_ = makeIconCursor( "package://rviz/icons/class/Waypoint" );

  service = n.advertiseService("static_map", &EditMapTool::mapCallback, this);
  //pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1,

  // Latched publisher for metadata
  metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
        // Latched publisher for data
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

  isEnableMouseFlag = false;
}

void EditMapTool::activate()
{
  state_ = START;
  isEnableMouseFlag = true;

}

void EditMapTool::deactivate()
{
}

int EditMapTool::processMouseEvent( ViewportMouseEvent& event )
{
int flags = 0;

  Ogre::Vector3 pos;

  std::stringstream ss;

  if (isEnableMouseFlag)
  {

      bool success = context_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, pos );


      switch ( state_)
      {
        case START:
          break;
        case END:
          if ( success )
          {
            line_->setPoints(start_,pos);
          }
          break;
      }

      ss << "Click on two points to set virtual obstacles. Right-click to reset.";
      //setStatus( QString( ss.str().c_str() ) );

      if( event.leftDown() && success )
      {
        switch ( state_)
        {
          case START:
            start_ = pos;
            state_ = END;
            break;
          case END:
            end_ = pos;
            state_ = START;
            line_->setPoints(start_,end_);
            getLinePoints(start_, end_);
            packetMapMsg();
            updateMap();
            line_->setVisible(false);
            break;
        }
      }

  /*    if ( event.rightUp() )
      {
        state_ = START;
        line_->setVisible(false);
      } */

      // Delete virtual obstacle
      if( event.type == QEvent::MouseMove && event.right() )
      {
        getRectPoints(pos);
        updateMap();
      }
      return flags;
  }
}


bool EditMapTool::mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      ROS_INFO("Sending map");

      return true;
    }


void EditMapTool::loadFileMap(const std::string& fname, double res)
{
  std::string mapfname = "";
  double origin[3];
  int negate;
  double occ_th, free_th;
  MapMode mode = TRINARY;
  std::string frame_id;
  ros::NodeHandle private_nh("~");
  private_nh.param("frame_id", frame_id, std::string("map"));
  deprecated = (res != 0);
  if (!deprecated) {
    //mapfname = fname + ".pgm";
    //std::ifstream fin((fname + ".yaml").c_str());
    std::ifstream fin(fname.c_str());
    if (fin.fail()) {
      ROS_ERROR("Map_server could not open %s.", fname.c_str());
      exit(-1);
    }
#ifdef HAVE_NEW_YAMLCPP
    // The document loading process changed in yaml-cpp 0.5.
    YAML::Node doc = YAML::Load(fin);
#else
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
#endif
    try {
      doc["resolution"] >> res;
    } catch (YAML::InvalidScalar) {
      ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
      exit(-1);
    }
    try {
      doc["negate"] >> negate;
    } catch (YAML::InvalidScalar) {
      ROS_ERROR("The map does not contain a negate tag or it is invalid.");
      exit(-1);
    }
    try {
      doc["occupied_thresh"] >> occ_th;
    } catch (YAML::InvalidScalar) {
      ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
      exit(-1);
    }
    try {
      doc["free_thresh"] >> free_th;
    } catch (YAML::InvalidScalar) {
      ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
      exit(-1);
    }
    try {
      std::string modeS = "";
      doc["mode"] >> modeS;

      if(modeS=="trinary")
        mode = TRINARY;
      else if(modeS=="scale")
        mode = SCALE;
      else if(modeS=="raw")
        mode = RAW;
      else{
        ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
        exit(-1);
      }
    } catch (YAML::Exception) {
      ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
      mode = TRINARY;
    }
    try {
      doc["origin"][0] >> origin[0];
      doc["origin"][1] >> origin[1];
      doc["origin"][2] >> origin[2];
    } catch (YAML::InvalidScalar) {
      ROS_ERROR("The map does not contain an origin tag or it is invalid.");
      exit(-1);
    }
    try {
      doc["image"] >> mapfname;
      // TODO: make this path-handling more robust
      if(mapfname.size() == 0)
      {
        ROS_ERROR("The image tag cannot be an empty string.");
        exit(-1);
      }
      if(mapfname[0] != '/')
      {
        // dirname can modify what you pass it
        char* fname_copy = strdup(fname.c_str());
        mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
        free(fname_copy);
      }
    } catch (YAML::InvalidScalar) {
      ROS_ERROR("The map does not contain an image tag or it is invalid.");
      exit(-1);
    }
  } else {
    private_nh.param("negate", negate, 0);
    private_nh.param("occupied_thresh", occ_th, 0.65);
    private_nh.param("free_thresh", free_th, 0.196);
    mapfname = fname;
    origin[0] = origin[1] = origin[2] = 0.0;
  }

  ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
  map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, mode);
  map_resp_.map.info.map_load_time = ros::Time::now();
  map_resp_.map.header.frame_id = frame_id;
  map_resp_.map.header.stamp = ros::Time::now();
  ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
           map_resp_.map.info.width,
           map_resp_.map.info.height,
           map_resp_.map.info.resolution);
  meta_data_message_ = map_resp_.map.info;

  m_mapMsg = map_resp_.map;
  updateMap();
}

// Modify the map data according to user selection
void EditMapTool::packetMapMsg()
{
  for (int i = 0; i < point2dList.size(); i++)
  {
    SPoint2d point = point2dList[i];
    m_mapMsg.data[point.y * m_mapMsg.info.width + point.x] = 100;
    //std::cout<<"point:"<<point.x <<" " << point.y<< std::endl;
    //std::cout<<"pix value"<< m_mapMsg.data[point.y * m_mapMsg.info.width + point.x] <<std::endl;
  }
}

void EditMapTool::updateMap()
{
  map_resp_.map = m_mapMsg;
  metadata_pub.publish( meta_data_message_ );
  map_pub.publish( m_mapMsg );
}

void EditMapTool::getRectPoints(Ogre::Vector3 centerPoint)
{
    float originX = map_resp_.map.info.origin.position.x;
  float originY = map_resp_.map.info.origin.position.y;
  point2dList.clear();
  int startX = floatToInt((centerPoint.x - originX ) / map_resp_.map.info.resolution); 
  int startY = floatToInt((centerPoint.y - originY ) / map_resp_.map.info.resolution);

  for (int i = -half_width; i < half_width; i ++)
  {
    for (int j = -half_width; j < half_width; j ++)
    {
      SPoint2d point;
      point.x = startX + i;
      point.y = startY + j;
      point2dList.push_back(point);
    }
  }
  for (int i = 0; i < point2dList.size(); i++)
  {
    SPoint2d point = point2dList[i];
    m_mapMsg.data[point.y * m_mapMsg.info.width + point.x] = 0;
    //std::cout<<"point:"<<point.x <<" " << point.y<< std::endl;
    //std::cout<<"pix value"<< m_mapMsg.data[point.y * m_mapMsg.info.width + point.x] <<std::endl;
  }
}

void EditMapTool::getLinePoints(Ogre::Vector3 startPoint, Ogre::Vector3 endPoint)
{
  point2dList.clear();

  float originX = map_resp_.map.info.origin.position.x;
  float originY = map_resp_.map.info.origin.position.y;

  float startX = (startPoint.x - originX ) / map_resp_.map.info.resolution; 
  float startY = (startPoint.y - originY ) / map_resp_.map.info.resolution;

  float endX = (endPoint.x - originX ) / map_resp_.map.info.resolution; 
  float endY =  (endPoint.y - originY ) / map_resp_.map.info.resolution;


  float lineSlope = (endY - startY) / (endX - startX);

  float angle = atan(lineSlope) * 180 / M_PI; 

  if (angle < 45 || angle > -45)
  {
    if (floatToInt(startX) < floatToInt(endX) )
    {
        for (int xi = floatToInt(startX); xi < floatToInt(endX); xi ++)
        {
          float yi = lineSlope * (xi - startX) + startY;

          SPoint2d point;
          point.x = xi;
          point.y = floatToInt(yi);
          point2dList.push_back(point);
          //std::cout<<"point point:"<<xi <<" " << point.y<< std::endl;
        }
    }
    else
    {
        for (int xi = floatToInt(endX); xi < floatToInt(startX); xi++)
        {
          float yi = lineSlope * (xi - startX) + startY;

          SPoint2d point;
          point.x = xi;
          point.y = floatToInt(yi);
          point2dList.push_back(point);
        }
    }
  }
  if ((angle > 45 && angle < 90) ||  (angle > -90 && angle < -45))
  {
    if (floatToInt(startX) < floatToInt(endX) )
    {
        for (int yi = floatToInt(startY); yi < floatToInt(endY); yi ++)
        {
          float xi = (1 / lineSlope) * (yi - startY) + startX;

          SPoint2d point;
          point.x = floatToInt(xi);
          point.y = yi;
          point2dList.push_back(point);
          //std::cout<<"point point:"<<xi <<" " << point.y<< std::endl;
        }
    }
    else
    {
        for (int yi = floatToInt(endY); yi < floatToInt(startY); yi ++)
        {
          float xi = (1 / lineSlope) * (yi - startY) + startX;

          SPoint2d point;
          point.x = floatToInt(xi);
          point.y = yi;
          point2dList.push_back(point);
          //std::cout<<"point point:"<<xi <<" " << point.y<< std::endl;
        }
    }
  }

}

// float to int conversion
int EditMapTool::floatToInt(float f)
{  
  int i = 0;  
  if(f>0) //正数  
    i = (f*10 + 5)/10;  
  else if(f<0) //负数  
    i = (f*10 - 5)/10;  
  else i = 0;  

  return i;  
} 


} /* namespace rviz */


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::EditMapTool, rviz::Tool )
