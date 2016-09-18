#ifndef LOAD_MAP_H
#define LOAD_MAP_H

#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/selection/selection_manager.h"

#include "rviz/visualization_manager.h"

#include <OgreVector3.h>

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include <string.h>

#include "rviz/ogre_helpers/line.h"
#include "rviz/load_resource.h"
#include "rviz/tool.h"

//using namespace map_server;
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
class LoadMap : Tool
{
  public:
    enum 
    {
      START,
      END
    } state;

    struct SPoint2d
    {
      int x;
      int y;
    };

    typedef std::vector<SPoint2d> Point2dList;

    /** Trivial constructor */



    LoadMap()
    {
      service = n.advertiseService("static_map", &LoadMap::mapCallback, this);
      //pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1,

      // Latched publisher for metadata
      metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
            // Latched publisher for data
      map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

      isEnableMouseFlag = false;
    }

    void loadFileMap(const std::string& fname, double res)
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

    void setVisualManager(VisualizationManager* vsiualManager)
    {
      m_visualManager = vsiualManager;
    }

    void activate()
    {
      state = START;
      isEnableMouseFlag = true;
    }

    void deactivate()
    {
      state = END;
    }

  private:
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    ros::ServiceServer service;
    bool deprecated;


    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      ROS_INFO("Sending map");

      return true;
    }

    /** The map data is cached here, to be sent out to service callers
     */
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;


    nav_msgs::OccupancyGrid m_mapMsg;

    VisualizationManager* m_visualManager;

    Ogre::Vector3 start_;
    Ogre::Vector3 end_;

    Point2dList point2dList;

    QCursor std_cursor_;
    QCursor hit_cursor_;
    Line* line_;

    bool isEnableMouseFlag;

    /*
    void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
    {
      pub.publish( meta_data_message_ );
    }
    */

  private:
    
    // Modify the map data according to user selection
    void packetMapMsg()
    {
      for (int i = 0; i < point2dList.size(); i++)
      {
        SPoint2d point = point2dList[i];
        m_mapMsg.data[point.y * m_mapMsg.info.width + point.x] = 1;
      }
//      memcpy(m_mapMsg.data, buffer, m_mapMsg.info.width * m_mapMsg.info.height);
    }

    void updateMap()
    {


      metadata_pub.publish( meta_data_message_ );
      map_pub.publish( m_mapMsg );
    }

    int processMouseEvent( ViewportMouseEvent& event )
    {
      int flags = 0;

      Ogre::Vector3 pos;

      std::stringstream ss;

      if (isEnableMouseFlag)
      {

          bool success = m_visualManager->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, pos );

          switch ( state)
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

          if( event.leftUp() && success )
          {
            switch ( state)
            {
              case START:
                start_ = pos;
                state = END;
                break;
              case END:
                end_ = pos;
                state = START;
                line_->setPoints(start_,end_);
                getLinePoints(start_, end_);
                packetMapMsg();
                updateMap();
                break;
            }
          }

          if ( event.rightUp() )
          {
            state = START;
            line_->setVisible(true);
          }
          return flags;
      }

    }

    void getLinePoints(Ogre::Vector3 startPoint, Ogre::Vector3 endPoint)
    {
      float startIntPosX = startPoint.x / map_resp_.map.info.resolution;
      float startIntPosY = startPoint.y / map_resp_.map.info.resolution;
      float endIntPosX   = endPoint.x / map_resp_.map.info.resolution;
      float endIntPosY   = endPoint.y / map_resp_.map.info.resolution;

      float lineSlope = (endPoint.y - startPoint.y) / (endPoint.x - startPoint.x);
      for (int xi = floatToInt(startIntPosX); xi < floatToInt(endIntPosX); xi ++)
      {
        float yi = lineSlope * (xi - startIntPosX) + startPoint.y;

        SPoint2d point;
        point.x = xi;
        point.y = floatToInt(yi);
        point2dList.push_back(point);
      }
    }

    // float to int conversion
    int floatToInt(float f)
    {  
      int i = 0;  
      if(f>0) //正数  
        i = (f*10 + 5)/10;  
      else if(f<0) //负数  
        i = (f*10 - 5)/10;  
      else i = 0;  
    
      return i;  
    } 

};

}
#endif // LOAD_MAP_H
