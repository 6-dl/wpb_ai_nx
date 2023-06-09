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
#include <std_msgs/String.h>
#include <wpb_ai_bringup/Coord.h>

void ObjCoordCB(const wpb_ai_bringup::Coord::ConstPtr &msg)
{
    int nNumObj = msg->name.size();
    ROS_WARN("[ObjCoordCB] obj = %d",nNumObj);
    if(nNumObj > 0)
    {
        for(int i=0 ; i<nNumObj ; i++)
        {
            ROS_WARN("[ObjCoordCB] %s - (%.2f , %.2f , %.2f)",msg->name[i].c_str(),msg->x[i],msg->y[i],msg->z[i]);
        }
        
    }
} 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_ai_obj_node");
    ROS_WARN("wpb_ai_obj_node start!");

    ros::NodeHandle n;
    ros::Subscriber obj_sub = n.subscribe("/yolo/coord", 1, ObjCoordCB);

    ros::spin();

    return 0;
}
