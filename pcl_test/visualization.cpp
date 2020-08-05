#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>   //深度图可视化的头文件
#include "visualization.h"


void
setViewerPose2(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0], up_vector[1], up_vector[2]);
}


void generatroRangeImage() {
	//pcl::PointCloud<pcl::PointXYZ> pointCloud;      //定义点云对象

	//for (float y = -0.5f; y <= 0.5f; y += 0.01f) {        //循环产生点数据

	//	for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
	//		pcl::PointXYZ point;
	//		point.x = 2.0f - y;
	//		point.y = y;
	//		point.z = z;
	//		pointCloud.points.push_back(point);          //循环添加点数据到点云对象
	//	}
	//}

	//pointCloud.width = (uint32_t)pointCloud.points.size();
	//pointCloud.height = 1;                            //设置点云对象的头信息


	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);

	// -----从创建的点云中获取深度图--//
	float angularResolution = (float)(1.0f * (M_PI / 180.0f)); // 按弧度1度
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f)); // 按弧度360.0度
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f)); // 按弧度180.0度
	// 传感器位置
	// 深度图采集的方向是从点云图的哪个角度？
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 2.0f, 0.0f);  //采集位置
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;     //深度图像遵循的坐标系统

	float noiseLevel = 0.00;
	float minRange = 0.0f;
	int borderSize = 1;

	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	/*
	 关于range_image.createFromPointCloud（）参数的解释 （涉及的角度都为弧度为单位） ：
	   point_cloud为创建深度图像所需要的点云
	  angular_resolution_x深度传感器X方向的角度分辨率
	  angular_resolution_y深度传感器Y方向的角度分辨率
	   pcl::deg2rad (360.0f)深度传感器的水平最大采样角度
	   pcl::deg2rad (180.0f)垂直最大采样角度
	   scene_sensor_pose设置的模拟传感器的位姿是一个仿射变换矩阵，默认为4*4的单位矩阵变换
	   coordinate_frame定义按照那种坐标系统的习惯  默认为CAMERA_FRAME
	   noise_level  获取深度图像深度时，邻近点对查询点距离值的影响水平
	   min_range 设置最小的获取距离，小于最小的获取距离的位置为传感器的盲区
	   border_size  设置获取深度图像边缘的宽度 默认为0
	*/
	range_image.createFromPointCloud(
		*cloud,
		angularResolution,
		angularResolution,
		pcl::deg2rad(360.0f),
		pcl::deg2rad(180.0f),
		sensorPose,
		coordinate_frame,
		noiseLevel,
		minRange,
		borderSize);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcForShow(cloud);
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(pcForShow, "pc");
	viewer.setCameraPosition(0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

	setViewerPose2(viewer, range_image.getTransformationToWorldSystem());  //设置视点的位置
	viewer.initCameraParameters();
	//可视化深度图
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(range_image);

	viewer.spin();
}

//int main() {
//	VisualLization::demoCallBack1();
//	//generatroRangeImage();
//	return 0;
//}