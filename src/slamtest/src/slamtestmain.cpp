#include <iostream>
#include <algorithm>

//ros
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

//pcl库
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>

//OpenCV
#include <opencv2/opencv.hpp>

#include <cmath>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


using namespace Eigen;
using namespace std;
using namespace cv;
using namespace pcl;

//函数功能：读雷达数据并转换为笛卡尔坐标系
MatrixXf ReadLisarData(std::string bagfile,float RotAngle){
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
        break;
    }
    
    MatrixXf v1 = MatrixXf::Ones(cart.rows(),1);
    MatrixXf cart1,Out1,Out2;
    cart1.resize(cart.rows(),4);
    cart1<<cart,v1;
    
    Out1=P*cart1.transpose();
    Out2=Out1.transpose();

    bag.close();    
    return Out2;
    
}

//函数功能：删除矩阵某一行
void RemoveRow(Eigen::MatrixXf& matrix, unsigned int rowToRemove){
  unsigned int numRows = matrix.rows() - 1;
  unsigned int numCols = matrix.cols();
 
  if( rowToRemove < numRows ) {
   matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) =
     matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);
  }
 
  matrix.conservativeResize(numRows,numCols);
}

//函数功能：找最近的不为0的像素点深度值
VectorXf FindNearestDepth(vector<Point2f> mc,MatrixXf mD){
    VectorXf depth;
    depth.resize(mc.size());
    for(int i=0;i<mc.size();i++)
    {
        for(int j=1;j<10;j++)
        {
            if(mD(mc[i].y-j,mc[i].x-j)!=0)
            {
                depth(i,0) = mD(mc[i].y-j,mc[i].x-j);
                break;
            }
            else if(mD(mc[i].y-j,mc[i].x)!=0)
            {
                depth(i,0) = mD(mc[i].y-j,mc[i].x);
                break;
            }
            else if(mD(mc[i].y,mc[i].x-j)!=0)
            {
                depth(i,0) = mD(mc[i].y,mc[i].x-j);
                break;
            }
            else if(mD(mc[i].y-j,mc[i].x+j)!=0)
            {
                depth(i,0) = mD(mc[i].y-j,mc[i].x+j);
                break;
            }
            else if(mD(mc[i].y+j,mc[i].x-j)!=0)
            {
                depth(i,0) = mD(mc[i].y+j,mc[i].x-j);
                break;
            }
            else if(mD(mc[i].y,mc[i].x+j)!=0)
            {
                depth(i,0) = mD(mc[i].y,mc[i].x+j);
                break;
            }
            else if(mc[i].y+j,mD(mc[i].x)!=0)
            {
                depth(i,0) = mD(mc[i].y+j,mc[i].x);
                break;
            }
            else if(mD(mc[i].y+j,mc[i].x+j)!=0)
            {
                depth(i,0) = mD(mc[i].y+j,mc[i].x+j);
                break;
            }
            
        }
    }
    return depth;
        
    
}

//函数功能：根据矩阵第三列元素对矩阵行排序
void sort_vec(const MatrixXf& mtrx,MatrixXf& sorted_mtrx,VectorXi& ind){  
  ind = VectorXi::LinSpaced(mtrx.rows(),0,mtrx.rows()-1);
  auto rule=[mtrx](int i,int j)->bool
  {
    return mtrx(i,2)<mtrx(j,2);
  };
  sort(ind.data(),ind.data()+ind.size(),rule);
  //data成员函数返回VectorXd的第一个元素的指针，类似于begin()
  sorted_mtrx.resize(mtrx.rows(),mtrx.cols());
  for(int i=0;i<mtrx.rows();i++){
    sorted_mtrx.row(i)=mtrx.row(ind(i));
  }
}

//函数功能：SQDIST
MatrixXf SQDIST(MatrixXf Sample,MatrixXf Pre){
    MatrixXf A,B,C;
    A.resize(Sample.rows(),Pre.rows());
    B.resize(Sample.rows(),Pre.rows());
    C.resize(Sample.rows(),Pre.rows());
    for(int i=0;i<Sample.cols();i++){
        for(int j=0;j<Pre.rows();j++){
            A.col(j)=Sample.col(i);
        }
        for(int k=0;k<Sample.rows();k++){
            B.row(k)=Pre.col(i).transpose();
        }
        C=C.array()+pow((A-B).array(),2);
    }
    return C;
}

struct GPRdatastruct{
    MatrixXf fmean;
    VectorXf V;
    
};

//函数功能：GPR
GPRdatastruct GPR(MatrixXf inputloghyper,MatrixXf x_oneobj,MatrixXf y_oneobj,MatrixXf star_oneobj){
    GPRdatastruct GPRdata;
    float ell=0,sf2=0,sn2=0;
    ell=inputloghyper(0);
    sf2=pow(inputloghyper(1),2);
    sn2=pow(inputloghyper(2),2);
    MatrixXf Kxx = MatrixXf::Zero(x_oneobj.rows(),x_oneobj.rows());
    ////加速计算方法，参考来自《Gaussian Processes for Machine Learning》
    MatrixXf Kx_pro = -0.5*SQDIST(x_oneobj.array()/ell,x_oneobj.array()/ell);
    MatrixXf KKxx = sf2*exp(Kx_pro.array());
    Kxx=Kxx+KKxx;
    KKxx=sn2*(MatrixXf::Identity(x_oneobj.rows(),x_oneobj.rows()));
    Kxx=Kxx+KKxx;
    MatrixXf L,alpha;
    L.resize(Kxx.rows(),Kxx.cols());
    alpha.resize(Kxx.rows(),1);
    L=Kxx.llt().matrixL();       //cholesky
    alpha=L.transpose().inverse()*(L.inverse()*y_oneobj);
    ////求预测和方差
    MatrixXf Kxxstar = MatrixXf::Zero(x_oneobj.rows(),star_oneobj.rows());
    MatrixXf Kxstar_pro = -0.5*SQDIST(x_oneobj.array()/ell,star_oneobj.array()/ell);
    MatrixXf KKxxstar=sf2*exp(Kxstar_pro.array());
    Kxxstar = Kxxstar + KKxxstar;
    GPRdata.fmean = Kxxstar.transpose()*alpha;
    MatrixXf v = L.inverse()*Kxxstar;
    MatrixXf vx = v.array()*v.array();
    GPRdata.V = 1-vx.colwise().sum().array();
    return GPRdata;
}


int main(int argc, char** argv){
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
    
    
    //Readdepthmap
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
	filter2D(I, imgLaplance, CV_32F, kernel, Point(-1, -1), 0);
	I.convertTo(sharpenImg, CV_32F);
	Mat resultImg = sharpenImg-imgLaplance;
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
	findContours(dist_8u, contours,RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    
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
	int obstacle_num = contours.size();
    cout<<"obstacle_num = "<<obstacle_num<<endl;
    /*******************************end****************************/
    
    //计算中心坐标
	vector<Moments> mu(obstacle_num);
    vector<Point2f> mc(obstacle_num);
	for (int i = 0; i < obstacle_num; i++)
	{
		mu[i] = moments(contours[i], false);                             //计算矩
        mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);   //计算中心矩
        mc[i].x=round(mc[i].x);
        mc[i].y=round(mc[i].y);
	}
    
    //寻找最近不为0的深度值作为中心点深度值
    VectorXf depth;
    depth = FindNearestDepth(mc,mD);
    
    //中心点坐标降序排序
    MatrixXf CTP,CTP_sorted;
    VectorXi Ind;
    CTP.resize(obstacle_num,3);
    CTP_sorted.resize(obstacle_num,3);
    for(int i=0;i<obstacle_num;i++){
        CTP(i,0) = mc[i].y;
        CTP(i,1) = mc[i].x;
        CTP(i,2) = depth(i);
    }
    sort_vec(CTP,CTP_sorted,Ind);
    cout<<"中心点坐标降序排序及对应深度值："<<endl<<CTP_sorted<<endl;
    
    //将当前凸包内部点标号设为1
    MatrixXf Label; 
    Label.resize(I.rows,I.cols);
    int rowsnum=0;
    for(int i=0;i<I.cols;i++){
        for(int j=0;j<I.rows;j++){
            if(int(mark.at<uchar>(j,i))==int(mark.at<uchar>(CTP(0,0),CTP(0,1))))
            {
                Label(j,i)=1;
                rowsnum++;
            }
            else
            {
                Label(j,i)=0;
            }
        }
    }
    //只留障碍物处深度值
    MatrixXf mDD;       //稀疏深度层
    mDD.resize(I.rows,I.cols);
    mDD = mD.array()*Label.array();
    
    //计算凸包内部点的坐标
    MatrixXf star_oneobj;
    int rowsnum2=0;
    star_oneobj.resize(rowsnum,2);
    for(int i=0;i<I.cols;i++){
        for(int j=0;j<I.rows;j++){
            if(Label(j,i)==1)
            {
                star_oneobj(rowsnum2,0)=i;
                star_oneobj(rowsnum2,1)=j;
                rowsnum2++;
            }
        }
    }
    
    //计算已知深度的坐标及深度值
    Mat pro_emg;
	resultImg.convertTo(pro_emg, CV_8U);
	vector<vector<Point>> contours2;
    vector<Vec4i> hierarchy;
	findContours(pro_emg, contours2,hierarchy,RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    VectorXf mark_objs;
    mark_objs.resize(Pxx.rows());
    for(int i=0;i<Pxx.rows();i++){
        Point2f pt;
        pt.x = Pxx(i,0);
        pt.y = Pxx(i,1);
        if(pointPolygonTest(contours2[Ind(0)],pt,false)==1){
            mark_objs(i)=1;
        }
        else{
            mark_objs(i)=0;
        }
    }
    MatrixXf x_oneobj,xx_oneobj,y_oneobj,yy_oneobj;
    x_oneobj.resize(Pxx.rows(),2);
    y_oneobj.resize(Pxx.rows(),1);
    x_oneobj.col(0)=Pxx.block(0,0,Pxx.rows(),1).array()*mark_objs.array();
    x_oneobj.col(1)=Pxx.block(0,1,Pxx.rows(),1).array()*mark_objs.array();
    y_oneobj=Pxx.col(2).array()*mark_objs.array();
    xx_oneobj=x_oneobj;
    yy_oneobj=y_oneobj;
    
    for(int i=0,j=0,k=0;i<Pxx.rows();i++){
        if(xx_oneobj(i,0)==0){
            RemoveRow(x_oneobj,j);
        }
        else{
            j++;
        }
        if(yy_oneobj(i,0)==0){
            RemoveRow(y_oneobj,k);
        }
        else{
            k++;
        }
    }
    float m_Z_oneobj=0;
    m_Z_oneobj=y_oneobj.sum()/y_oneobj.rows();       //平均值
    y_oneobj=y_oneobj.array()-m_Z_oneobj;
    
    
    //GPR
    MatrixXf inputloghyper;
    inputloghyper.resize(3,1);
    inputloghyper<<1,1,0.05f;
    GPRdatastruct GPRdata;
    GPRdata=GPR(inputloghyper,x_oneobj,y_oneobj,star_oneobj);
    MatrixXf mu_oneobj,Zl_oneobj;
    mu_oneobj=GPRdata.fmean.array()+m_Z_oneobj;
    Zl_oneobj=y_oneobj.array()+m_Z_oneobj;

    float now_uncertainty_value = GPRdata.V.sum()/GPRdata.V.rows()*100; //计算不确定度副
    cout<<"当前目标不确定度:"<<now_uncertainty_value<<endl;
    
    //RANSAC移除地面
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
    cloud->width = velox.rows();
    cloud->height = 1;
    cloud->points.resize(velox.rows());
    for(int i=0;i<velox.rows();i++)
    {
        cloud->points[i].x=velox(i,0);
        cloud->points[i].y=velox(i,1);
        cloud->points[i].z=velox(i,2);
    }
    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr inliers (new PointIndices);
    SACSegmentation<PointXYZ> seg;            // Create the segmentation object
    seg.setOptimizeCoefficients (true);       // Optional
    seg.setModelType (SACMODEL_PLANE);        // Mandatory
    seg.setMethodType (SAC_RANSAC);
    seg.setDistanceThreshold (0.018f);         // 距离阈值 单位m
    seg.setInputCloud (cloud);
    seg.segment(*inliers, *coefficients);
    ExtractIndices<PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_filtered);
    
    //cout<<cloud_filtered->points.size()<<endl;
    /*//pcl移除地面点云显示
    pcl::visualization::CloudViewer viewer("Filtered");
    viewer.showCloud(cloud_filtered);
    while(!viewer.wasStopped()){
    
    }
    */
    
    //聚类算法
    search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
    tree->setInputCloud (cloud_filtered);
    
    vector<PointIndices> cluster_indices;
    EuclideanClusterExtraction<PointXYZ> ec;
    ec.setClusterTolerance (0.023f);       //设置近邻搜索的搜索半径为2.5cm
    ec.setMinClusterSize (30);             //设置一个聚类需要的最少点数目为30
    ec.setMaxClusterSize (400);            //设置一个聚类需要的最大点数目
    ec.setSearchMethod (tree);             //设置点云的搜索机制
    ec.setInputCloud (cloud_filtered);     //设置原始点云 
    ec.extract (cluster_indices);          //从点云中提取聚类
    
    //可视化聚类结果
    int color_bar[][3]=
    {
        { 255,0,0},
        { 0,255,0 },
        { 0,0,255 },
        { 0,255,255 },
        { 255,255,0 },
        { 255,255,255 },
        { 255,0,255 }
    };
    visualization::PCLVisualizer viewer("segmention");
    int j = 0;
    vector <float> moment_of_inertia;
    vector <float> eccentricity;
    PointXYZ min_point_AABB;             //AABB包围盒
    PointXYZ max_point_AABB;
    Vector3f major_vector, middle_vector, minor_vector;
    Vector3f mass_center;
    MatrixXf Centroid_Point_velo;
    Centroid_Point_velo.resize(obstacle_num,4);
    for(vector<PointIndices>::const_iterator it=cluster_indices.begin();it!=cluster_indices.end();++it)
    {
        PointCloud<PointXYZ>::Ptr cloud_cluster (new PointCloud<PointXYZ>);
        for(vector<int>::const_iterator pit=it->indices.begin();pit != it->indices.end();pit++)
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]); 
        
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cout<<"PointCloud representing the Cluster: "<<cloud_cluster->points.size()<<" data points."<< endl;
        
        //计算包围盒
        MomentOfInertiaEstimation <PointXYZ> feature_extractor;
        feature_extractor.setInputCloud(cloud_cluster);
        feature_extractor.compute();
        
        
        feature_extractor.getMomentOfInertia(moment_of_inertia);
        feature_extractor.getEccentricity(eccentricity);
        feature_extractor.getAABB(min_point_AABB, max_point_AABB);
        feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter(mass_center);

        cout<<"包围盒"<<j+1<<"边长:"<<endl;
        cout<<max_point_AABB.x-min_point_AABB.x<<"|"<<max_point_AABB.y-min_point_AABB.y<<"|"
            <<max_point_AABB.z-min_point_AABB.z<<endl;
        cout<<"包围盒"<<j+1<<"中心点:"<<endl;
        cout<<mass_center<<endl;
        
        Centroid_Point_velo(j,0)=mass_center(0);
        Centroid_Point_velo(j,1)=mass_center(1);
        Centroid_Point_velo(j,2)=mass_center(2);
        Centroid_Point_velo(j,3)=1;
    
        /*//可视化包围盒
        visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_in_color_h(cloud,
            color_bar[j][0],
            color_bar[j][1],
            color_bar[j][2]);//赋予显示点云的颜色
        viewer.addPointCloud(cloud_cluster, cloud_in_color_h, to_string(j));
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, to_string(j));
        viewer.addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, 
                       max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, to_string(j));
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,            
                                           visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, to_string(j));
        */
        j++;
    }
    cout<<"定位完成，开始重建"<<endl;
    
    
    
    
    
    
    
    //3D重建部分
    ////计算中心点深度排序
    MatrixXf centerpoint_px,centerpoint_px_pro1;
    centerpoint_px.resize(obstacle_num,3);
    centerpoint_px_pro1 = P*Centroid_Point_velo.transpose();
    centerpoint_px=centerpoint_px_pro1.transpose();
    centerpoint_px.col(0)=centerpoint_px.col(0).array()/centerpoint_px.col(2).array();
    centerpoint_px.col(1)=centerpoint_px.col(1).array()/centerpoint_px.col(2).array();
    MatrixXf centerpoint_px_sorted;
    VectorXi Ind2,Label_up_new;
    centerpoint_px_sorted.resize(obstacle_num,3);
    Ind2.resize(obstacle_num);
    Label_up_new.resize(obstacle_num);
    sort_vec(centerpoint_px,centerpoint_px_sorted,Ind2);
    for(int i=0;i<obstacle_num;i++)
    {
        Label_up_new(i)=int(mark.at<uchar>(round(centerpoint_px_sorted(i,1)),round(centerpoint_px_sorted(i,0))));
    }
    
    ////开始依次重建
    for(int i1=0;i1<obstacle_num;i1++)
    {
        //当前目标处设为1
        int labelnum=0;
        for(int i=0;i<I.cols;i++){
            for(int j=0;j<I.rows;j++){
                if(int(mark.at<uchar>(j,i))==Label_up_new(i1))
                {
                    Label(j,i)=1;
                    labelnum++;
                }
                else
                {
                    Label(j,i)=0;
                }
            }
        }
        //计算凸包内部点的坐标
        MatrixXf xstar_manybj;
        int labelnum2=0;
        xstar_manybj.resize(labelnum,2);
        for(int i=0;i<I.cols;i++){
            for(int j=0;j<I.rows;j++){
                if(Label(j,i)==1)
                {
                    xstar_manybj(labelnum2,0)=i;
                    xstar_manybj(labelnum2,1)=j;
                    labelnum2++;
                }
            }
        }
        
        //
        //计算已知深度的坐标及深度值
        VectorXf mark_objs2;
        mark_objs2.resize(Pxx.rows());
        for(int i=0;i<Pxx.rows();i++){
            Point2f pt;
            pt.x = Pxx(i,0);
            pt.y = Pxx(i,1);
            if(pointPolygonTest(contours2[i1],pt,false)==1){
                mark_objs2(i)=1;
            }
            else{
                mark_objs2(i)=0;
            }
        }
        MatrixXf x_manyobj,xx_manyobj,y_manyobj,yy_manyobj;
        x_manyobj.resize(Pxx.rows(),2);
        y_manyobj.resize(Pxx.rows(),1);
        x_manyobj.col(0)=Pxx.block(0,0,Pxx.rows(),1).array()*mark_objs2.array();
        x_manyobj.col(1)=Pxx.block(0,1,Pxx.rows(),1).array()*mark_objs2.array();
        y_manyobj=Pxx.col(2).array()*mark_objs2.array();
        xx_manyobj=x_manyobj;
        yy_manyobj=y_manyobj;
        
        for(int i=0,j=0,k=0;i<Pxx.rows();i++){
            if(xx_manyobj(i,0)==0){
                RemoveRow(x_manyobj,j);
            }
            else{
                j++;
            }
            if(yy_manyobj(i,0)==0){
                RemoveRow(y_manyobj,k);
            }
            else{
                k++;
            }
        }
        float m_Z_manyobj=0;
        m_Z_manyobj=y_manyobj.sum()/y_manyobj.rows();       //平均值
        y_manyobj=y_manyobj.array()-m_Z_manyobj;
        
        //GPR
        GPRdatastruct GPRdata2;
        GPRdata2=GPR(inputloghyper,x_manyobj,y_manyobj,xstar_manybj);
        MatrixXf mu_oneobj,Zl_oneobj;
        mu_oneobj=GPRdata2.fmean.array()+m_Z_manyobj;
        Zl_oneobj=y_manyobj.array()+m_Z_manyobj;
        
        
        /*drawContours( pro_emg, contours2, 0, 1, CV_FILLED, 8, hierarchy );
        imshow("wind",pro_emg);
        cout<<mark<<endl;
        cout<<Label<<endl;
        cout<<x_manyobj<<endl;
        cout<<y_manyobj<<endl;
        cout<<Zl_oneobj<<endl;
        cout<<Ind2<<endl;
        cout<<i1<<endl;
        cout<<centerpoint_px_sorted<<endl;
        cout<<Label_up_new<<endl;
        */
        break;
    }
    
    /*//pcl可视化
    while(!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    */
    
    
    
    waitKey(0);
    
    return 0;
    
}
