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
 * measure_tool.cpp
 *
 *  Created on: Sep 11, 2016
 *      Author: Samuel
 */

#include <qdebug.h>
#include "rviz/default_plugin/tools/path_follow_tool.h"

#include <tf/transform_listener.h>
#include <actionlib_msgs/GoalID.h>

#include <geometry_msgs/PoseStamped.h>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

#include "rviz/geometry.h"

#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"
#include "rviz/render_panel.h"

namespace rviz
{

PathFollowTool::PathFollowTool()
{
    shortcut_key_ = 'f';

}

PathFollowTool::~PathFollowTool()
{

}

void PathFollowTool::onInitialize()
{
  setName( "Follow Path" );

//  hit_cursor_ = makeIconCursor( "package://rviz/icons/classes/Pose.png" );
}

void PathFollowTool::activate()
{
  setStatus( "Click mouse to set path points." );
  qDebug()<<"ACTIVATE";
  state_ = Position;
  result_sub_   = nh_.subscribe("move_base/result", 10, &PathFollowTool::onGoalReached,this);
  start_pos_sub_ = nh_.subscribe("/amcl_pose", 10, &PathFollowTool::onCrtPoseGeted,this);

  goal_pub_   = nh_.advertise<geometry_msgs::PoseStamped>( "/move_base_simple/goal", 1 );
  result_pub_ = nh_.advertise<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1);
//  cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);

  right_has_down_ = false;
  start_pos_has_added_ = false;
  goal_list_.clear();
}

void PathFollowTool::deactivate()
{
  qDebug() << "Deactivate";
  right_has_down_ = false;
  start_pos_has_added_ = false;
  result_sub_.shutdown();
  start_pos_sub_.shutdown();
  //cancel the move_base, it can't be put here, as I don't know how to restart move_base
//  cancel_pub_.publish(actionlib_msgs::GoalID());


  while(!arrow_list_.empty())
  {
    arrow_list_.back()->getSceneNode()->setVisible( false );
    delete arrow_list_.back();
    arrow_list_.pop_back();
  }
}

void PathFollowTool::onGoalReached(const move_base_msgs::MoveBaseActionResult& a)
{
    if (goal_idx_ == 0)
        direction_ = forward;
    else if(goal_idx_ == goal_list_size_-1)
        direction_ = backward;


    if (direction_ == forward)
    {
        goal_idx_++;
        publishGoal(goal_list_.at(goal_idx_));

    }
    else if(direction_ == backward )
    {
        goal_idx_--;
        GoalPose goal;
        goal.x = goal_list_.at(goal_idx_).x;
        goal.y = goal_list_.at(goal_idx_).y;
        goal.theta = goal_list_.at(goal_idx_+1).theta + Ogre::Math::PI;

        publishGoal(goal);
    }
}

void PathFollowTool::onCrtPoseGeted(const geometry_msgs::PoseWithCovarianceStamped& amcl_pos)
{
    start_pos_.x = amcl_pos.pose.pose.position.x;
    start_pos_.y = amcl_pos.pose.pose.position.y;
    start_pos_.theta = tf::getYaw(amcl_pos.pose.pose.orientation);
}

int PathFollowTool::processMouseEvent( ViewportMouseEvent& event )
{
  int flags = 0;

  // all mouse event will not be responded once right button has been down.
  if (right_has_down_)
  {
    flags |= (Render);
    return flags;
  }

  if( event.leftDown() )
  {
//    ROS_ASSERT( state_ == Position );

    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
    if( getPointOnPlaneFromWindowXY( event.viewport,
                                     ground_plane,
                                     event.x, event.y, intersection ))
    {
      pos_ = intersection;
      arrow_list_.push_back( new Arrow( scene_manager_, NULL, 2.0f, 0.2f, 0.5f, 0.35f ) );
      arrow_list_.back()->setColor( 0.12f, 0.56f, 1.0f, 1.0f );
      arrow_list_.back()->getSceneNode()->setVisible( true );
      arrow_list_.back()->setPosition( pos_ );
      arrow_list_.back()->setOrientation( Ogre::Quaternion( Ogre::Radian(Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z ) );

      state_ = Orientation;
      flags |= Render;
    }
  }
//  else if( event.type == QEvent::MouseMove && event.left() )
//  {
//    if( state_ == Orientation )
//    {
//      //compute angle in x-y plane
//      Ogre::Vector3 cur_pos;
//      Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
//      if( getPointOnPlaneFromWindowXY( event.viewport,
//                                       ground_plane,
//                                       event.x, event.y, cur_pos ))
//      {
//        double angle = atan2( cur_pos.y - pos_.y, cur_pos.x - pos_.x );

//        arrow_list_.back()->getSceneNode()->setVisible( true );

//        //we need base_orient, since the arrow goes along the -z axis by default (for historical reasons)
//        Ogre::Quaternion orient_x = Ogre::Quaternion( Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y );

//        arrow_list_.back()->setOrientation( Ogre::Quaternion( Ogre::Radian(angle), Ogre::Vector3::UNIT_Z ) * orient_x );

//        flags |= Render;
//      }
//    }
//  }
  else if( event.leftUp() )
  {
//    if( state_ == Orientation )
//    {
//      //compute angle in x-y plane
//      Ogre::Vector3 cur_pos;
//      Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
//      if( getPointOnPlaneFromWindowXY( event.viewport,
//                                       ground_plane,
//                                       event.x, event.y, cur_pos ))
//      {
        if(!start_pos_has_added_)
        {
            start_pos_has_added_ = true;
            onPoseSet(start_pos_.x, start_pos_.y, 0); //start_pos_.theta
        }
        GoalPose last_pos = goal_list_.back();
        double angle = atan2( pos_.y - last_pos.y, pos_.x - last_pos.x );
        onPoseSet(pos_.x, pos_.y, angle);

//        flags |= (Finished|Render);
        flags |= (Render);
//      }
//    }
  }
  else if( event.rightDown() && !right_has_down_ )
  {
    right_has_down_ = true;
    move_base_msgs::MoveBaseActionResult result;

    //    publish fake result topic
    result_pub_.publish(result);
    goal_list_size_ = goal_list_.size();
    goal_idx_ = 0;

    flags |= (Render);
  }
  return flags;
}

void PathFollowTool::onPoseSet(double x, double y, double theta)
{
    GoalPose Goal;
    Goal.x = x;
    Goal.y = y;
    Goal.theta = theta;
    goal_list_.push_back(Goal);
}

void PathFollowTool::publishGoal(GoalPose& GoalPose)
{
    std::string fixed_frame = context_->getFixedFrame().toStdString();
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, GoalPose.theta);
    tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(GoalPose.x, GoalPose.y, 0.0)), ros::Time::now(), fixed_frame);
    geometry_msgs::PoseStamped goal;
    tf::poseStampedTFToMsg(p, goal);
    ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
      goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, GoalPose.theta);

    goal_pub_.publish(goal);
}


} /* namespace rviz */


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::PathFollowTool, rviz::Tool )
