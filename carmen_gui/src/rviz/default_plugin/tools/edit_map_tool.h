/*
 *  The tool is used to edit the map 
 *
 *  Created on:  2016.08.31
 *      Author: bob
 */

 #ifndef EDIT_MAP_TOOL_H_
#define EDIT_MAP_TOOL_H_

#include "rviz/tool.h"

#include <OgreVector3.h>
  #include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
#include "yaml-cpp/yaml.h"

const int half_width = 3;

namespace rviz
{
class Line;

class EditMapTool : public Tool
{
public:
	 enum 
    {
      START,
      END
    } state_;

    struct SPoint2d
    {
      int x;
      int y;
    };

    typedef std::vector<SPoint2d> Point2dList;
  EditMapTool();
  virtual
  ~EditMapTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( ViewportMouseEvent& event );

  int saveMap();

    virtual void loadFileMap(const std::string& fname, double res);

  void cancelPublish()
  {
  	map_pub.shutdown();
  	metadata_pub.shutdown();
  }
private:

  private:
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    ros::ServiceServer service;
    bool deprecated;

     /** The map data is cached here, to be sent out to service callers
     */
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;
    nav_msgs::OccupancyGrid m_mapMsg;
    Ogre::Vector3 start_;
    Ogre::Vector3 end_;
    Point2dList point2dList;
    QCursor std_cursor_;
    QCursor hit_cursor_;
    Line* line_;
    float length_;

    bool isEnableMouseFlag;

private:

    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res );
    void packetMapMsg();

    void getLinePoints(Ogre::Vector3 startPoint, Ogre::Vector3 endPoint);

    void getRectPoints(Ogre::Vector3 centerPoint);

    void updateMap();

    int floatToInt(float f);

};

} /* namespace rviz */
#endif /* EDIT_MAP_TOOL_H_ */