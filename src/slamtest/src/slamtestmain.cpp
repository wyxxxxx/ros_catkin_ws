#include <iostream>

//ros
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// 此处为include相关消息类的头文件，如果有自定义的头文件，请将其包含在内
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace Eigen;
//"/home/wyx/ros_catkin_ws/lidardata/0.bag"
MatrixXf ReadLisarData(std::string bagfile)
{
    //打开bag文件
    rosbag::Bag bag;
    bag.open(bagfile,rosbag::bagmode::Read); 
    
    //设置需要遍历的topic
    std::vector<std::string> topics; 
    topics.push_back(std::string("/scan"));         
    
    //创建view，用于读取bag中的topic
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    
    float dist[721] = {0};
    float cartAngles[721] = {0};
    float cartarray[1442]={0};
    MatrixXf cart;
    
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::LaserScan::ConstPtr s = m.instantiate<sensor_msgs::LaserScan>();
        
        int j=0;
        for(int i=0;i<721;i++)
        {
            if((s->ranges[i]!=INFINITY)&&(s->ranges[i]>=s->range_min)&&(s->ranges[i]<=s->range_max))
            {
                dist[j]=s->ranges[i];
                cartAngles[j]=-1.5708+i*0.00436333f;
                j++;
            }
                
        }
        std::cout<<j<<std::endl;
        
        for(int a=0;a<j;a++)
        {
            float x,y;
            x=cos(cartAngles[a])*dist[a];
            y=sin(cartAngles[a])*dist[a];
            cartarray[a]=x;
            cartarray[(j+a)]=y;
        }
        
    
        cart = Map<MatrixXf>(cartarray,j,2);
        std::cout<<cart<<std::endl;
        break;
    }
    
    bag.close();
    
    return cart;
    
}






int main(int argc, char** argv)
{
    //初始化ROS
    ros::init (argc, argv, "slamtest");
    
    MatrixXf bag1;
    bag1=ReadLisarData("/home/wyx/ros_catkin_ws/lidardata/1.bag");

    
    return 0;
    
}
