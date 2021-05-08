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
MatrixXf ReadLisarData(std::string bagfile,float RotAngle)
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
    float cartarray[2163]={0};
    MatrixXf cart;
    
    //旋转矩阵参数
    float L = 0.13;  //单线激光雷达扫描中心与云台旋转中心的距离
    int alpha = 0;   //单线激光雷达扫描中心与云台旋转中心之间的连线与雷达z轴负方向的夹角
    
    float RotationAngle1 = RotAngle*EIGEN_PI/180.0f;
    Matrix3f R1;
    R1<<cos(RotationAngle1),0,-sin(RotationAngle1),0,1,0,sin(RotationAngle1),0,cos(RotationAngle1);
    Vector3f T1;
    T1<<-2*L*sin(RotationAngle1/2.0f)*cos(alpha-RotationAngle1/2.0f),0,-2*L*sin(RotationAngle1/2.0f)*sin(RotationAngle1/2.0f-alpha);
    Matrix4f R1T1,P;
    R1T1<<R1,T1,0,0,0,1;
    P=R1T1.inverse();
    
//    std::cout<<R1<<std::endl;
//    std::cout<<T1<<std::endl;
//    std::cout<<R1T1<<std::endl;
//    std::cout<<P<<std::endl;
    
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::LaserScan::ConstPtr s = m.instantiate<sensor_msgs::LaserScan>();
        
        int j=0;
        float x,y;
        for(int i=0;i<721;i++)
        {
            if((s->ranges[i]!=INFINITY)&&(s->ranges[i]>=s->range_min)&&(s->ranges[i]<=s->range_max))
            {
                dist[j]=s->ranges[i];
                cartAngles[j]=-1.5708+i*0.00436333f;
                x=cos(cartAngles[j])*dist[j];
                if(x>0)
                {
                    j++;
                }
            }
                
        }
//        std::cout<<j<<std::endl;
        
        int rownum = j;
        
        for(int a=0,b=0;a<j;a++)
        {
            x=cos(cartAngles[a])*dist[a];
            y=sin(cartAngles[a])*dist[a];
            cartarray[b]=x;
            cartarray[(rownum+b)]=y;
            b++;
            
        }
    
        cart = Map<MatrixXf>(cartarray,rownum,3);
//        std::cout<<cart<<std::endl;
        break;
    }
    
    MatrixXf v1 = MatrixXf::Ones(cart.rows(),1);
    MatrixXf cart1,Out1,Out2;
    cart1.resize(cart.rows(),4);
    cart1<<cart,v1;
    
    Out1=P*cart1.transpose();
    Out2=Out1.transpose();
    
//  std::cout<<Out2<<std::endl;
    
    
    bag.close();
    
    return Out2;
    
}


//函数功能：删除矩阵某一行
void RemoveRow(Eigen::MatrixXf& matrix, unsigned int rowToRemove) {
  unsigned int numRows = matrix.rows() - 1;
  unsigned int numCols = matrix.cols();
 
  if( rowToRemove < numRows ) {
   matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) =
     matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);
  }
 
  matrix.conservativeResize(numRows,numCols);
}





int main(int argc, char** argv)
{
    //初始化ROS
    ros::init (argc, argv, "slamtest");
    
    MatrixXf velo6,velo7,velo8,velo10,velo11,velo12,velo13,velo14,velo15,velo16,velo17,velo18;
    velo6  = ReadLisarData("/home/wyx/ros_catkin_ws/lidardata/6.bag",6);
    velo7  = ReadLisarData("/home/wyx/ros_catkin_ws/lidardata/7.bag",7);
    velo8  = ReadLisarData("/home/wyx/ros_catkin_ws/lidardata/8.bag",8);
    velo10 = ReadLisarData("/home/wyx/ros_catkin_ws/lidardata/10.bag",10);
    velo11 = ReadLisarData("/home/wyx/ros_catkin_ws/lidardata/11.bag",11);
    velo12 = ReadLisarData("/home/wyx/ros_catkin_ws/lidardata/12.bag",12);
    velo13 = ReadLisarData("/home/wyx/ros_catkin_ws/lidardata/13.bag",13);
    velo14 = ReadLisarData("/home/wyx/ros_catkin_ws/lidardata/14.bag",14);
    velo15 = ReadLisarData("/home/wyx/ros_catkin_ws/lidardata/15.bag",15);
    velo16 = ReadLisarData("/home/wyx/ros_catkin_ws/lidardata/16.bag",16);
    velo17 = ReadLisarData("/home/wyx/ros_catkin_ws/lidardata/17.bag",17);
    velo18 = ReadLisarData("/home/wyx/ros_catkin_ws/lidardata/18.bag",18);
    
    MatrixXf velo,velox;
    velo.resize((velo6.rows()+velo7.rows()+velo8.rows()+velo10.rows()+velo11.rows()+velo12.rows()
                  +velo13.rows()+velo14.rows()+velo15.rows()+velo16.rows()+velo17.rows()+velo18.rows()),4);
    velo<<velo13,
          velo10,
          velo11,
          velo14,
          velo15,
          velo16,
          velo17,
          velo18,
          velo7,
          velo8,
          velo6,
          velo12;

    velox=velo;
    for(int i=0,j=0;i<velo.rows();i++)
    {
        if((velo(i,0)>0.8f)||(velo(i,1)<-0.25)||(velo(i,1)>0.4f))
        {
            RemoveRow(velox,j);
        }
        else
        {
            j++;
        }
    }
    
    
    return 0;
    
}
