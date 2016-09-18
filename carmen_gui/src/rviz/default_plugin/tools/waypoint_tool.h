/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 /*
 * waypoint_tool.h
 *
 *  Created on: Sep 11, 2016
 *      Author: Samuel
 */

#ifndef RVIZ_WAYPOINT_TOOL_H
#define RVIZ_WAYPOINT_TOOL_H

#include <QObject>
#include <OgreVector3.h>
#include <QCursor>

#include <ros/ros.h>

#include "rviz/default_plugin/tools/pose_tool.h"
#include "rviz/ogre_helpers/arrow.h"
#include <vector>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tf/tf.h"

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

namespace rviz
{
class Arrow;
class DisplayContext;
class StringProperty;

struct GoalPose
{
    double x;
    double y;
    double theta;
};

enum MoveDirect
{
    forward,
    backward
};

class WaypointTool : public PoseTool
{
Q_OBJECT
public:
  WaypointTool();
  virtual ~WaypointTool();
  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( ViewportMouseEvent& event );
  void publishGoal(GoalPose&);

protected:
  virtual void onPoseSet(double x, double y, double theta);
  void onGoalReached(const move_base_msgs::MoveBaseActionResult&);
  void onCrtPoseGeted(const geometry_msgs::PoseWithCovarianceStamped&amcl_pose);

private:
  ros::NodeHandle nh_;
  ros::Publisher goal_pub_;
  ros::Publisher result_pub_;
  ros::Publisher cancel_pub_;
  ros::Subscriber result_sub_;
  ros::Subscriber start_pos_sub_;
  std::vector<Arrow*> arrow_list_;
  std::vector<GoalPose> goal_list_;
  int goal_list_size_;
  int goal_idx_;
  bool right_has_down_;
  bool start_pos_has_added_;
  MoveDirect direction_;
  GoalPose start_pos_;

  StringProperty* topic_property_;
};

} /* namespace rviz */
#endif /* RVIZ_WAYPOINT_TOOL_H */
