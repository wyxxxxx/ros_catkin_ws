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

/*//pcl库
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
pcl::visualization::CloudViewer viewer("Cloud Viewer");
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
*/

//OpenCV
#include <opencv2/opencv.hpp>

#include <cmath>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


using namespace Eigen;
using namespace std;
using namespace cv;


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
//        cout<<j<<endl;
        
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
//        cout<<cart<<endl;
        break;
    }
    
    MatrixXf v1 = MatrixXf::Ones(cart.rows(),1);
    MatrixXf cart1,Out1,Out2;
    cart1.resize(cart.rows(),4);
    cart1<<cart,v1;
    
    Out1=P*cart1.transpose();
    Out2=Out1.transpose();
    
//  cout<<Out2<<endl;
    
    
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
    //参数设置
    //相机内参矩阵
    Matrix<float,3,4> R0;  
    R0<<347.3107f,0,341.4781f,0,0,347.1215f,186.9693f,0,0,0,1,0;   
        
    //RT外参
    float Z_theta=(EIGEN_PI/180)*0;   
    float y_beta=(EIGEN_PI/180)*0;
    float x_alpha=(EIGEN_PI/180)*0;
    float T_x=0;
    float T_y=-0.18;
    float T_z=-0.04;
        
    Matrix3f R,R_0,Rx,Ry,Rz;
    R_0<<0,-1,0,0,0,-1,1,0,0;
    Rx<<1,0,0,0,cos(x_alpha),-sin(x_alpha),0,sin(x_alpha),cos(x_alpha);
    Ry<<cos(y_beta),0,sin(y_beta),0,1,0,-sin(y_beta),0,cos(y_beta);
    Rz<<cos(Z_theta),-sin(Z_theta),0,sin(Z_theta),cos(Z_theta),0,0,0,1;
    R=R_0*Rx*Ry*Rz;
        
    Vector3f T;
    T<<T_x,
    T_y,
    T_z;
    Matrix4f RT;
    RT<<R,T,
        0,0,0,1;
    
    //初始化ROS
    ros::init (argc, argv, "slamtest");
    
    //读点云数据
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
          
    //筛选出一部分点
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
    
    /*//点云显示数据定义
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    cloud->points.resize(velox.rows());
    cloud->width = velox.rows();
    cloud->height = 1;
    
    for(int i=0;i<velox.rows();i++)
    {
        cloud->points[i].x=velox(i,0);
        cloud->points[i].y=velox(i,1);
        cloud->points[i].z=velox(i,2);
    }
    */
    /*//pcl库点云图像显示
    viewer.showCloud(cloud);
    while (!viewer.wasStopped ())
    {
    }
    */
    /*//rviz点云显示
    ros::NodeHandle nh; 
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1); 
    sensor_msgs::PointCloud2 output; 
    pcl::toROSMsg(*cloud, output); 
	output.header.frame_id = "odom"; 
    
    ros::Rate loop_rate(1); 
	while (ros::ok()) 
	{ 
		pcl_pub.publish(output);
	    ros::spinOnce(); 
		loop_rate.sleep(); 
	 }
	 */ 

    //读取图像
    Mat I = imread("/home/wyx/ros_catkin_ws/src/opencvtest/picturedata/manyobs1.jpg");
    if (I.empty()) {
		printf("could not load image...\n");
		return -1;
	}
    
    Matrix<float,3,4> P;
    P=R0*RT;
    MatrixXf Px,Pxx;
    Px.resize(3,velox.rows());
    Px=P*velox.transpose();
    Pxx.resize(velox.rows(),3);
    Pxx=Px.transpose();
    
    Pxx.col(0)=Pxx.col(0).array()/Pxx.col(2).array();
    Pxx.col(1)=Pxx.col(1).array()/Pxx.col(2).array();
    
    //原始深度图
    MatrixXf mD = MatrixXf::Zero(I.rows,I.cols);
    for(int i=0;i<Pxx.rows();i++)
    {
        mD(round(Pxx(i,1)),round(Pxx(i,0)))=Pxx(i,2);
    }
    
    Mat GrayI;
    cvtColor(I,GrayI,CV_BGR2GRAY);
    Mat DImage = Mat::zeros(I.size(),CV_8UC1);
    
    /*//GrayI灰度图像显示
    namedWindow("GrayI", CV_WINDOW_AUTOSIZE);
	imshow("GrayI", GrayI);
    */
    
    MatrixXf dmap = MatrixXf::Zero(I.rows,I.cols);
    for(int i=0;i<mD.rows();i++)
    {
        for(int j=0;j<mD.cols();j++)
        {
            if(mD(i,j)!=0)
                dmap(i,j)=1/mD(i,j);
        }
    }
    dmap=255*(dmap.array()-dmap.minCoeff())/(dmap.maxCoeff()-dmap.minCoeff());
    dmap=round(dmap.array());
    
    for(int i=0;i<DImage.rows;i++)
    {
        for(int j=0;j<DImage.cols;j++)
        {
            DImage.at<uchar>(i,j)=0.5f*dmap(i,j)+ 0.5f*GrayI.at<uchar>(i,j);
        }
    }
    
    /*//DImage点云在图像上的投影显示
    namedWindow("DImage", CV_WINDOW_AUTOSIZE);
	imshow("DImage", DImage);
    */
    
    /******************************图像分割部分****************************/
    // 1. change background
	for (int row = 0; row < I.rows; row++) {
		for (int col = 0; col < I.cols; col++) {
			if ((I.at<Vec3b>(row, col)[0]>210)&&(I.at<Vec3b>(row, col)[1]>210)&&(I.at<Vec3b>(row, col)[2]>210)) {
				I.at<Vec3b>(row, col)[0] = 0;
				I.at<Vec3b>(row, col)[1] = 0;
				I.at<Vec3b>(row, col)[2] = 0;
			}
		}
	}
	Mat kernel = (Mat_<float>(3, 3) << 1, 1, 1, 1, -8, 1, 1, 1, 1);
	Mat imgLaplance;
	Mat sharpenImg = I;
	filter2D(I, imgLaplance, CV_32F, kernel, Point(-1, -1), 0, BORDER_DEFAULT);
	I.convertTo(sharpenImg, CV_32F);
	Mat resultImg = sharpenImg - imgLaplance;
	resultImg.convertTo(resultImg, CV_8UC3);
	imgLaplance.convertTo(imgLaplance, CV_8UC3);
	//imshow("sharpen image", resultImg);
    
    // convert to binary
	Mat binaryImg;
	cvtColor(I, resultImg, CV_BGR2GRAY);
	threshold(resultImg, binaryImg, 1, 255, THRESH_BINARY);
	//imshow("binary image", binaryImg);

	Mat distImg;
	distanceTransform(binaryImg, distImg, DIST_L1, 3, 5);
	normalize(distImg, distImg, 0, 1, NORM_MINMAX);
	//imshow("distance result", distImg);
    
    // binary again
	threshold(distImg, distImg, .4, 1, THRESH_BINARY);
	Mat k1 = Mat::ones(13, 13, CV_8UC1);
	erode(distImg, distImg, k1, Point(-1, -1));
	//imshow("distance binary image", distImg);
    
    // markers 
	Mat dist_8u;
	distImg.convertTo(dist_8u, CV_8U);
	vector<vector<Point>> contours;
	findContours(dist_8u, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    // create makers
	Mat markers = Mat::zeros(I.size(), CV_32SC1);
	for (size_t i = 0; i < contours.size(); i++) {
		drawContours(markers, contours, static_cast<int>(i), Scalar::all(static_cast<int>(i) + 1), -1);
	}
	circle(markers, Point(5, 5), 3, Scalar(255, 255, 255), -1);
	//imshow("my markers", markers*10000);

	// perform watershed
	watershed(I, markers);
	Mat mark = Mat::zeros(markers.size(), CV_8UC1);
	markers.convertTo(mark, CV_8UC1);
	bitwise_not(mark, mark, Mat());
	//imshow("watershed image", mark);

	// generate random color
	vector<Vec3b> colors;
	for (size_t i = 0; i < contours.size(); i++) {
		int r = theRNG().uniform(0, 255);
		int g = theRNG().uniform(0, 255);
		int b = theRNG().uniform(0, 255);
		colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
	}

	// fill with color and display final result
	Mat dst = Mat::zeros(markers.size(), CV_8UC3);
	for (int row = 0; row < markers.rows; row++) {
		for (int col = 0; col < markers.cols; col++) {
			int index = markers.at<int>(row, col);
			if (index > 0 && index <= static_cast<int>(contours.size())) {
				dst.at<Vec3b>(row, col) = colors[index - 1];
			}
			else {
				dst.at<Vec3b>(row, col) = Vec3b(0, 0, 0);
			}
		}
	}
	//imshow("Final Result", dst);
	
	//输出目标数量
    cout<<"obstacle_num = "<<contours.size()<<endl;
    /*******************************end****************************/
    
    //计算中心坐标
	vector<Moments> mu(contours.size());
    vector<Point2f> mc(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);                             //计算矩
        mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);   //计算中心矩
	}
	cout<<mc<<endl;

    waitKey(0);
    return 0;
    
}
