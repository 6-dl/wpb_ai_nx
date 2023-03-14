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
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <wpb_ai_bringup/Bbox.h>
#include <wpb_ai_bringup/Coord.h>

using namespace cv;

static Mat image_color;
static Mat image_predict;
static ros::Publisher predict_pub;
static cv_bridge::CvImage img_bridge;
static sensor_msgs::Image predict_img_msg;
static bool flag_predicted = false;

static ros::Publisher image_pub;
static std_msgs::String image_msg;
static std::string image_filename = "/dev/shm/raw.jpg";

static ros::Publisher pc_pub;
static tf::TransformListener *tf_listener; 
static ros::Publisher marker_pub;
static ros::Publisher coord_pub;
static visualization_msgs::Marker line_box;
static visualization_msgs::Marker text_marker;

void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB);
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
void RemoveBoxes();

typedef struct stObjectBox
{
    std::string name;
    int left;
    int right;
    int top;
    int bottom;
    float probability;
}stObjectBox;

static std::vector<stObjectBox> objects;
static std::vector<stObjectBox>::const_iterator object_iter;

void callbackColorImage(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("callbackColorImage");
    if(flag_predicted == false)
        return;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_ptr->image.copyTo(image_color);
    imwrite(image_filename,image_color);
    image_msg.data = image_filename;
    image_pub.publish(image_msg);
    flag_predicted = false;
    //ROS_WARN("raw image  %s",image_msg.data .c_str()); 
}

void callbackBbox(const wpb_ai_bringup::Bbox &msg)
{
    objects.clear();
    int nNum = msg.name.size();
    if(nNum > 0)
    {
        std::vector<stObjectBox> recv_objects;
        stObjectBox box_object;
        for(int i = 0; i < nNum; i++)
        {
            box_object.name = msg.name[i];
            box_object.left = msg.left[i];
            box_object.right = msg.right[i];
            box_object.top = msg.top[i];
            box_object.bottom = msg.bottom[i];
            box_object.probability = msg.probability[i];
            recv_objects.push_back(box_object);
        }
        objects = recv_objects;
    }
}

void callbackInfo(const std_msgs::String &msg)
{
    ROS_WARN("[callbackInfo] ");
    int nFindIndex = 0;
    nFindIndex = msg.data.find("start");
    if( nFindIndex >= 0 )
    {
        flag_predicted = true;
        ROS_WARN("[yolo_start] ");
    }
}

static int counter =0;
void callbackPredict(const std_msgs::String &msg)
{
    //ROS_WARN("predict image  %s",msg.data .c_str()); 
    image_predict = imread(msg.data);
    std_msgs::Header header;
    counter ++;
    header.seq = counter;
    header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image_predict);
    img_bridge.toImageMsg(predict_img_msg);
    predict_pub.publish(predict_img_msg); 
    flag_predicted = true;
}

void callbackPointCloud(const sensor_msgs::PointCloud2 &input)
{
    std::vector<stObjectBox> arObject = objects;
    if (arObject.size() <= 0) 
    {
        return;
    }
    
     //to footprint
    sensor_msgs::PointCloud2 pc_footprint;
    bool res = tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(5.0)); 
    if(res == false)
    {
        return;
    }
    pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    //source cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint , cloud_src);
    //ROS_WARN("cloud_src size = %d  width = %d",cloud_src.size(),input.width); 

    wpb_ai_bringup::Coord coord;
    // Draw Boxes
    RemoveBoxes();
    int object_index = 0;
    std::vector<stObjectBox>::const_iterator i;
    for (object_iter = arObject.begin(); object_iter != arObject.end(); ++object_iter) 
    {
        int rgb_object_x = (object_iter->left  + object_iter->right)/2;
        int rgb_object_y = (object_iter->top + object_iter->bottom)/2;
        int index_pc = rgb_object_y*input.width + rgb_object_x;

        float object_x = cloud_src.points[index_pc].x;
        float object_y = cloud_src.points[index_pc].y;
        float object_z = cloud_src.points[index_pc].z;

        // 扩散寻找有效坐标点
        for(int nc=0;nc<100;nc++)
        {
            int tmp_index = index_pc;
            object_x = cloud_src.points[tmp_index].x;
            if(std::fpclassify(object_x) != FP_NAN)
            {
                object_y = cloud_src.points[tmp_index].y;
                object_z = cloud_src.points[tmp_index].z;
                break;
            }

            tmp_index = index_pc-nc;
            if(tmp_index>=0 && tmp_index < 1920*1080)
            {
                object_x = cloud_src.points[tmp_index].x;
                if(std::fpclassify(object_x) != FP_NAN)
                {
                    object_y = cloud_src.points[tmp_index].y;
                    object_z = cloud_src.points[tmp_index].z;
                    break;
                }
            }

            tmp_index = index_pc+nc;
            if(tmp_index>=0 && tmp_index < 1920*1080)
            {
                object_x = cloud_src.points[tmp_index].x;
                if(std::fpclassify(object_x) != FP_NAN)
                {
                    object_y = cloud_src.points[tmp_index].y;
                    object_z = cloud_src.points[tmp_index].z;
                    break;
                }
            }

            tmp_index = index_pc-1920*nc;
            if(tmp_index>=0 && tmp_index < 1920*1080)
            {
                object_x = cloud_src.points[tmp_index].x;
                if(std::fpclassify(object_x) != FP_NAN)
                {
                    object_y = cloud_src.points[tmp_index].y;
                    object_z = cloud_src.points[tmp_index].z;
                    break;
                }
            }

            tmp_index = index_pc+1920*nc;
            if(tmp_index>=0 && tmp_index < 1920*1080)
            {
                object_x = cloud_src.points[tmp_index].x;
                if(std::fpclassify(object_x) != FP_NAN)
                {
                    object_y = cloud_src.points[tmp_index].y;
                    object_z = cloud_src.points[tmp_index].z;
                    break;
                }
            }
        }

        if(object_x > 0 && object_x < 1.7)
        {
            // 在疑似目标物有效坐标点附近滤波出点云
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_ptr;
            cloud_source_ptr = cloud_src.makeShared(); 
            pcl::PassThrough<pcl::PointXYZRGB> pass;//设置滤波器对象
            pass.setInputCloud (cloud_source_ptr);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (object_z-0.1, object_z+0.3);
            pass.filter (*cloud_source_ptr);
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (object_x-0.2, object_x+0.1);
            pass.filter (*cloud_source_ptr);
            pass.setFilterFieldName ("y");
            pass.setFilterLimits (object_y-0.2, object_y+0.1);
            pass.filter (*cloud_source_ptr);

            // 检测平面
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
            segmentation.setInputCloud(cloud_source_ptr);
            segmentation.setModelType(pcl::SACMODEL_PLANE);
            segmentation.setMethodType(pcl::SAC_RANSAC);
            segmentation.setDistanceThreshold(0.005);
            segmentation.setOptimizeCoefficients(true);
            Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);
            segmentation.setAxis(axis);
            segmentation.setEpsAngle(  10.0f * (M_PI/180.0f) );
            pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
            segmentation.segment(*planeIndices, *coefficients);
            
            // 获取平面高度
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud (cloud_source_ptr);
            extract.setIndices (planeIndices);
            extract.setNegative (false);
            extract.filter (*plane);
            float plane_height = plane->points[0].z;
            int points_num = plane->points.size();
            for(int i=0;i<points_num;i++)
            {
                if(plane->points[i].z > plane_height)
                {
                    plane_height = plane->points[i].z;
                }
            }
            ROS_INFO("[obj_ind= %d] - plane: %d points. height =%.2f" ,object_index, plane->width * plane->height,plane_height);

            // 剔除平面,只留物品
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);
            pass.setInputCloud (cloud_source_ptr);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (plane_height+0.04, plane_height+0.3);
            pass.filter (*objects);

            // 找平面上的物品
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            std::vector<pcl::PointIndices> cluster_indices;
            ec.setClusterTolerance (0.04);
            ec.setMinClusterSize (200);
            ec.setMaxClusterSize (10000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (objects);
            ec.extract (cluster_indices);

            // 找出点数最多的物品
            pcl::ExtractIndices<pcl::PointXYZRGB> extract_object_indices;
            int obj_num = cluster_indices.size();
            if(obj_num == 0)
                continue;
            int max_index = 0;
            int max_size = 0;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            extract_object_indices.setInputCloud(objects);
            extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(cluster_indices[max_index]));
            extract_object_indices.filter(*object_cloud);
            max_size = object_cloud->points.size();
            for(int i = 1; i<obj_num; ++i)
            {
                extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(cluster_indices[i]));
                extract_object_indices.filter(*object_cloud);
                if(object_cloud->points.size() > max_size)
                {
                    max_index = i;
                    max_size = object_cloud->points.size();
                }
            }

            // 对点数最多的物品进行坐标值统计
            float xMax,xMin,yMax,yMin,zMax,zMin;
            extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(cluster_indices[max_index]));
            extract_object_indices.filter(*object_cloud);
            bool bFirstPoint = true;
            for (int j = 0; j < object_cloud->points.size(); j++) 
            {
                pcl::PointXYZRGB p = object_cloud->points[j];
                if(bFirstPoint == true)
                {
                    xMax = xMin = p.x;
                    yMax = yMin = p.y;
                    zMax = zMin = p.z;
                    bFirstPoint = false;
                }

                if(p.x < xMin) { xMin = p.x;}
                if(p.x > xMax) { xMax = p.x;}
                if(p.y < yMin) { yMin = p.y;}
                if(p.y > yMax) { yMax = p.y;}
                if(p.z < zMin) { zMin = p.z;}
                if(p.z > zMax) { zMax = p.z;}
            }
            object_x = (xMax + xMin)/2;
            object_y = (yMax + yMin)/2;
            object_z = (zMax + zMin)/2;

            // 将物品结果发布出去
            coord.name.push_back(object_iter->name);
            coord.x.push_back(object_x);
            coord.y.push_back(object_y);
            coord.z.push_back(object_z);
            coord.probability.push_back(object_iter->probability);

            object_index ++;

            float obj_width = 0.03;
            float obj_height = 0.1;
            //DrawBox(object_x-obj_width , object_x+obj_width, object_y-obj_width, object_y+obj_width, object_z-obj_height, object_z+obj_height, 0, 1, 0);
            DrawBox(xMin, xMax, yMin, yMax, zMin, zMax, 0, 1, 0);
            DrawText(object_iter->name,0.08, object_x, object_y, object_z + 0.12, 1,0,1);
        }

        ROS_WARN("%s (%d,%d) - (%.2f %.2f %.2f) - %.2f",object_iter->name.c_str(), rgb_object_x,rgb_object_y,object_x,object_y,object_z,object_iter->probability); 
    }
    //marker_pub.publish(line_box);
    coord_pub.publish(coord);

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
    text_marker.action = 3;
    marker_pub.publish(text_marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_ai_yolo_3d");
    ros::NodeHandle nh_param("~");
    ROS_WARN("wpb_ai_yolo_3d");

    tf_listener = new tf::TransformListener(); 
    ros::NodeHandle nh;
    ros::Subscriber rgb_sub = nh.subscribe("/kinect2/qhd/image_color", 1 , callbackColorImage);
    ros::Subscriber pc_sub = nh.subscribe("/kinect2/qhd/points", 1 , callbackPointCloud);
    ros::Subscriber yolo_sub = nh.subscribe("/yolo/bbox", 1 , callbackBbox);
    ros::Subscriber info_sub = nh.subscribe("/yolo/info", 1 , callbackInfo);
    ros::Subscriber predict_sub = nh.subscribe("/yolo/predict", 1 , callbackPredict);

    marker_pub = nh.advertise<visualization_msgs::Marker>("obj_marker", 1);
    coord_pub = nh.advertise<wpb_ai_bringup::Coord>("/yolo/coord", 10);
    image_pub = nh.advertise<std_msgs::String>("/yolo/input_filename", 1);
    predict_pub = nh.advertise<sensor_msgs::Image>("/yolo/image_predict", 1);

    ros::Rate loop_rate(30);
    while( ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete tf_listener; 

    return 0;
}
