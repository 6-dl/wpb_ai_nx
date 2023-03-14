/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/
#include <ros/ros.h>
#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <waterplus_map_tools/Waypoint.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <wpb_ai_bringup/Coord.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static geometry_msgs::Pose grab_msg;
static bool bRecoObj = false;
static bool bObjPose = false;
static bool bGrabDone = false;

void ObjCoordCB(const wpb_ai_bringup::Coord::ConstPtr &msg)
{
    if(bRecoObj == true)
    {
        int nNumObj = msg->name.size();
        for(int i=0; i<nNumObj; i++)
        {
            if(msg->name[i] == "water")
            {
                ROS_WARN("[ObjCoordCB] Found water (%.2f , %.2f , %.2f)",msg->x[i],msg->y[i],msg->z[i]);
                grab_msg.position.x = msg->x[i];
                grab_msg.position.y = msg->y[i];
                grab_msg.position.z = msg->z[i];
                bObjPose = true;
            }
        }
    }
}

void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "done")
    {
        bGrabDone = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_ai_wp_node");

    ros::NodeHandle nh;
    
    ros::Subscriber obj_sub = nh.subscribe("/yolo/coord", 1, ObjCoordCB);
    ros::Publisher grab_pub = nh.advertise<geometry_msgs::Pose>("/wpb_ai/grab_pose", 1);
    ros::Subscriber res_sub = nh.subscribe("/wpb_ai/grab_result", 30, GrabResultCB);

    ros::ServiceClient cliGetWPName = nh.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    waterplus_map_tools::GetWaypointByName srvName;
    srvName.request.name = "Rack";
    if (cliGetWPName.call(srvName))
    {
        std::string name = srvName.response.name;
        float x = srvName.response.pose.position.x;
        float y = srvName.response.pose.position.y;
        ROS_INFO("Waypoint: name = %s (%.2f,%.2f)", name.c_str(),x,y);
    }
    else
    {
        ROS_ERROR("Failed to call service get_waypoint_name");
    }

    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = srvName.response.pose;
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Arrived at Rack!");
        bRecoObj = true;
    }
    else
    {
        return 0;
    }
    
    while (bObjPose == false && ros::ok())
    {
         ros::spinOnce();
    }
    ROS_INFO("Start grabbing!");
    grab_pub.publish(grab_msg);
    bRecoObj = false;
    bGrabDone = false;

    while (bGrabDone == false && ros::ok())
    {
         ros::spinOnce();
    }

    srvName.request.name = "Delivery";
    if (cliGetWPName.call(srvName))
    {
        std::string name = srvName.response.name;
        float x = srvName.response.pose.position.x;
        float y = srvName.response.pose.position.y;
        ROS_INFO("Waypoint: name = %s (%.2f,%.2f)", name.c_str(),x,y);
    }
    else
    {
        ROS_ERROR("Failed to call service get_waypoint_name");
    }

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = srvName.response.pose;
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Arrived at Delivery!");
        bRecoObj = true;
    }
}
 