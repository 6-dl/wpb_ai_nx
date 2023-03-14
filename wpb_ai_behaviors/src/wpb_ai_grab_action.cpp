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
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
 
// 抓取参数调节（单位：米）(Modified in wpb_ai.yaml!!)
static float grab_y_offset = 0.0f;          //抓取前，对准物品，机器人的横向位移偏移量
static float grab_lift_offset = 0.0f;       //手臂抬起高度的补偿偏移量
static float grab_forward_offset = 0.0f;    //手臂抬起后，机器人向前抓取物品移动的位移偏移量
static float grab_gripper_value = 0.032;    //抓取物品时，手爪闭合后的手指间距

#define STEP_WAIT           0
#define STEP_FIND_PLANE     1
#define STEP_PLANE_DIST     2
#define STEP_FIND_OBJ       3
#define STEP_OBJ_DIST       4
#define STEP_HAND_UP        5
#define STEP_FORWARD        6
#define STEP_GRAB           7
#define STEP_OBJ_UP         8
#define STEP_BACKWARD       9
#define STEP_DONE           10
static int nStep = STEP_WAIT;

static std::string pc_topic;
static ros::Publisher pc_pub;
static ros::Publisher marker_pub;
static ros::Publisher vel_pub;
static ros::Publisher mani_ctrl_pub;
static sensor_msgs::JointState mani_ctrl_msg;
static ros::Publisher result_pub;

void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB);
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
void DrawPath(float inX, float inY, float inZ);
void RemoveBoxes();
void VelCmd(float inVx , float inVy, float inTz);

static visualization_msgs::Marker line_box;
static visualization_msgs::Marker line_follow;
static visualization_msgs::Marker text_marker;

static std_msgs::String result_msg;

static ros::Publisher odom_ctrl_pub;
static std_msgs::String ctrl_msg;
static geometry_msgs::Pose2D pose_diff;

static float fObjGrabX = 0;
static float fObjGrabY = 0;
static float fObjGrabZ = 0;
static float fMoveTargetX = 0;
static float fMoveTargetY = 0;

typedef struct stBoxMarker
{
    float xMax;
    float xMin;
    float yMax;
    float yMin;
    float zMax;
    float zMin;
}stBoxMarker;

static stBoxMarker boxMarker;
static stBoxMarker boxPlane;
static stBoxMarker obj_to_track;

static int nTimeDelayCounter = 0;

static float fPlaneDist = 0;
static float fTargetPlaneDist = 0.6;    //与桌子之间的目标距离
static float fTargetGrabX = 0.9;        //抓取时目标物品的x坐标
static float fTargetGrabY = 0.0;        //抓取时目标物品的y坐标
      

void GrabPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // 目标物品的坐标
    fObjGrabX = msg->position.x;
    fObjGrabY = msg->position.y;
    fObjGrabZ = msg->position.z;
    ROS_WARN("[OBJ_TO_GRAB] x = %.2f y= %.2f ,z= %.2f " ,fObjGrabX, fObjGrabY, fObjGrabZ);
    ctrl_msg.data = "pose_diff reset";
    odom_ctrl_pub.publish(ctrl_msg);

    // ajudge the dist to obj
    fMoveTargetX = fObjGrabX - fTargetGrabX;
    fMoveTargetY = fObjGrabY - fTargetGrabY + grab_y_offset;
    ROS_WARN("[MOVE_TARGET] x = %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);
    nStep = STEP_OBJ_DIST;
}

void PoseDiffCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    pose_diff.x = msg->x;
    pose_diff.y = msg->y;
    pose_diff.theta = msg->theta;
}

void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB)
{
    line_box.header.frame_id = "base_footprint";
    line_box.ns = "line_box";
    line_box.action = visualization_msgs::Marker::ADD;
    line_box.id = 0;
    line_box.type = visualization_msgs::Marker::LINE_LIST;
    line_box.scale.x = 0.005;
    line_box.color.r = inR;
    line_box.color.g = inG;
    line_box.color.b = inB;
    line_box.color.a = 1.0;

    geometry_msgs::Point p;
    p.z = inMinZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.z = inMaxZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);
    marker_pub.publish(line_box);
}

static int nTextNum = 2;
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "base_footprint";
    text_marker.ns = "line_obj";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = nTextNum;
    nTextNum ++;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = inScale;
    text_marker.color.r = inR;
    text_marker.color.g = inG;
    text_marker.color.b = inB;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = inX;
    text_marker.pose.position.y = inY;
    text_marker.pose.position.z = inZ;
    
    text_marker.pose.orientation=tf::createQuaternionMsgFromYaw(1.0);

    text_marker.text = inText;

    marker_pub.publish(text_marker);
}

void RemoveBoxes()
{
    line_box.action = 3;
    line_box.points.clear();
    marker_pub.publish(line_box);
    line_follow.action = 3;
    line_follow.points.clear();
    marker_pub.publish(line_follow);
    text_marker.action = 3;
    marker_pub.publish(text_marker);
}

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = msg->data.find("grab stop");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[grab_stop] ");
        nStep = STEP_WAIT;
        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = 0;
        vel_pub.publish(vel_cmd);
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_ai_grab_action");
    ROS_INFO("wpb_ai_grab_action");

    ros::NodeHandle nh;
    
    marker_pub = nh.advertise<visualization_msgs::Marker>("obj_marker", 10);

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    mani_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpb_ai/mani_ctrl", 30);
    result_pub = nh.advertise<std_msgs::String>("/wpb_ai/grab_result", 30);

    ros::Subscriber sub_grab_pose = nh.subscribe("/wpb_ai/grab_pose", 1, GrabPoseCallback);
    ros::Subscriber sub_beh = nh.subscribe("/wpb_ai/behaviors", 1, BehaviorCB);
    odom_ctrl_pub = nh.advertise<std_msgs::String>("/wpb_ai/ctrl", 30);
    ros::Subscriber pose_diff_sub = nh.subscribe("/wpb_ai/pose_diff", 1, PoseDiffCallback);

    mani_ctrl_msg.name.resize(2);
    mani_ctrl_msg.position.resize(2);
    mani_ctrl_msg.velocity.resize(2);
    mani_ctrl_msg.name[0] = "lift";
    mani_ctrl_msg.name[1] = "gripper";
    mani_ctrl_msg.position[0] = 0;
    mani_ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
    mani_ctrl_msg.position[1] = 0.16;
    mani_ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)

    ros::NodeHandle nh_param("~");
    nh_param.getParam("grab/grab_y_offset", grab_y_offset);
    nh_param.getParam("grab/grab_lift_offset", grab_lift_offset);
    nh_param.getParam("grab/grab_forward_offset", grab_forward_offset);
    nh_param.getParam("grab/grab_gripper_value", grab_gripper_value);

    ros::Rate r(30);
    while(nh.ok())
    {
    
        //4、左右平移对准目标物品 
        if(nStep == STEP_OBJ_DIST)
        {
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2;
            vy = (fMoveTargetY - pose_diff.y)/2;

            VelCmd(vx,vy,0);
            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(ctrl_msg);
                nTimeDelayCounter = 0;
                nStep = STEP_HAND_UP;
            }

            result_msg.data = "object x";
            result_pub.publish(result_msg);
        }

        //5、抬起手臂
        if(nStep == STEP_HAND_UP)
        {
            if(nTimeDelayCounter == 0)
            {
                mani_ctrl_msg.position[0] = fObjGrabZ + grab_lift_offset;
                mani_ctrl_msg.position[1] = 0.16;
                mani_ctrl_pub.publish(mani_ctrl_msg);
                ROS_WARN("[STEP_HAND_UP] lift= %.2f  gripper= %.2f " ,mani_ctrl_msg.position[0], mani_ctrl_msg.position[1]);
                result_msg.data = "hand up";
                result_pub.publish(result_msg);
            }
            nTimeDelayCounter ++;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            VelCmd(0,0,0);
            if(nTimeDelayCounter > 15*30)
            {
                fMoveTargetX = fTargetGrabX -0.65 + grab_forward_offset;
                fMoveTargetY = 0;
                ROS_WARN("[STEP_FORWARD] x = %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);
                nTimeDelayCounter = 0;
                //ctrl_msg.data = "pose_diff reset";
                //odom_ctrl_pub.publish(ctrl_msg);
                nStep = STEP_FORWARD;
            }
        }

         //6、前进靠近物品
        if(nStep == STEP_FORWARD)
        {
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2;
            vy = (fMoveTargetY - pose_diff.y)/2;

            VelCmd(vx,vy,0);

            //ROS_INFO("[STEP_FORWARD] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(ctrl_msg);
                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_GRAB] grab_gripper_value = %.2f",grab_gripper_value);
                nStep = STEP_GRAB;
            }

            result_msg.data = "forward";
            result_pub.publish(result_msg);
        }

        //7、抓取物品
        if(nStep == STEP_GRAB)
        {
            if(nTimeDelayCounter == 0)
            {
                result_msg.data = "grab";
                result_pub.publish(result_msg);
            }
            mani_ctrl_msg.position[1] = grab_gripper_value;      //抓取物品手爪闭合宽度
            mani_ctrl_pub.publish(mani_ctrl_msg);
            //ROS_WARN("[STEP_GRAB] lift= %.2f  gripper= %.2f " ,mani_ctrl_msg.position[0], mani_ctrl_msg.position[1]);

            nTimeDelayCounter++;
            VelCmd(0,0,0);
            if(nTimeDelayCounter > 8*30)
            {
                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_OBJ_UP]");
                nStep = STEP_OBJ_UP;
            }
        }

        //8、拿起物品
        if(nStep == STEP_OBJ_UP)
        {
            if(nTimeDelayCounter == 0)
            {
                mani_ctrl_msg.position[0] += 0.03;
                mani_ctrl_pub.publish(mani_ctrl_msg);
                //ROS_WARN("[MANI_CTRL] lift= %.2f  gripper= %.2f " ,mani_ctrl_msg.position[0], mani_ctrl_msg.position[1]);
                result_msg.data = "object up";
                result_pub.publish(result_msg);
            }
            nTimeDelayCounter++;
            VelCmd(0,0,0);
            if(nTimeDelayCounter > 3*30)
            {
                fMoveTargetX = -(fObjGrabX -0.55 + grab_forward_offset);
                fMoveTargetY = -(fObjGrabY + grab_y_offset);
                ROS_WARN("[STEP_BACKWARD] x= %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);

                nTimeDelayCounter = 0;
                nStep = STEP_BACKWARD;
            }
        }

        //9、带着物品后退
        if(nStep == STEP_BACKWARD)
        {
            //ROS_WARN("[STEP_BACKWARD] nTimeDelayCounter = %d " ,nTimeDelayCounter);
            //nTimeDelayCounter++;
            
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2;
            vy = (fMoveTargetY - pose_diff.y)/2;

            VelCmd(vx,vy,0);

            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(ctrl_msg);
                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_DONE]");
                nStep = STEP_DONE;
            }

            result_msg.data = "backward";
            result_pub.publish(result_msg);
        }

        //10、抓取任务完毕
        if(nStep == STEP_DONE)
        {
            if(nTimeDelayCounter < 10)
            {
                VelCmd(0,0,0);
                nTimeDelayCounter ++;
            }
            result_msg.data = "done";
            result_pub.publish(result_msg);
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}