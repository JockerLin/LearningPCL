#pragma once
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include "pointcloudfilter.h"

// 去除原始点云采集下的背景数据
int
planeSegment()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// ------------------------使用自定义数据------------------------
	//// Fill in the cloud data
	//cloud->width = 15;
	//cloud->height = 1;
	//cloud->points.resize(cloud->width * cloud->height);

	//// Generate the data
	//for (auto& point : *cloud)
	//{
	//	point.x = 1024 * rand() / (RAND_MAX + 1.0f);
	//	point.y = 1024 * rand() / (RAND_MAX + 1.0f);
	//	point.z = 1.0;
	//}

	//// Set a few outliers
	//(*cloud)[0].z = 2.0;
	//(*cloud)[3].z = -2.0;
	//(*cloud)[6].z = 4.0;
	//(*cloud)[7].z = -5.0;

	// ------------------------使用table数据------------------------
	/*pcl::io::loadPCDFile("examplemodel/table_scene_lms400.pcd", *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_src(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudFilter::voxelGridFilter(cloud, filter_src, 0.008);
	cloud = filter_src;*/

	// ------------------------使用手机数据------------------------
	pcl::io::loadPCDFile("examplemodel/part2_all.pcd", *cloud);

	// 定义平面分割参数
	std::cerr << "Point cloud data: " << cloud->size() << " points" << std::endl;
	/*for (const auto& point : *cloud)
		std::cerr << "    " << point.x << " "
		<< point.y << " "
		<< point.z << std::endl;*/

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	//pcl::SACMODEL_LINE
	// 选择分割类型为平面
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.5);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return (-1);
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;
	cout <<"coefficients size:"<< coefficients->values.size() << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

	std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
	//for (std::size_t i = 0; i < inliers->indices.size(); ++i)
	
	for (const auto& idx : inliers->indices)
	{
		cloud_plane->push_back(cloud->points[idx]);
		if (0 == idx % 500) {
			cout << "idx:" << idx << endl;
		}
		/*std::cerr << "    " << idx << "    " << cloud->points[idx].x << " "
			<< cloud->points[idx].y << " "
			<< cloud->points[idx].z << std::endl;*/
	}
		
	cout << "filter size:" << cloud_plane->points.size();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_plane, 255, 0, 0);
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud);
	viewer.addPointCloud(cloud_plane, red, "cloud_plane");
	viewer.spin();

	return (0);
}

// 分割提取桌面上的杯子 圆柱模型提取
int cylinderSegment() {

	// 总体步骤：滤波器提取出桌子杯子=>拟合平面分割桌子=>剔除平面拟合杯子=>得到最终圆柱

	// All the objects needed
	pcl::PCDReader reader;
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::PCDWriter writer;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	pcl::visualization::PCLVisualizer viewer;

	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

	// Read in the cloud data
	reader.read("examplemodel/table_scene_mug_stereo_textured.pcd", *cloud);
	std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

	// Build a passthrough filter to remove spurious NaNs
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;
	viewer.addPointCloud(cloud_filtered);
	viewer.spin();
	viewer.removeAllPointClouds();

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment(*inliers_plane, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	// 根据index来filter点云，不要用for的土办法
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	// Write the planar inliers to disk
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_plane);
	viewer.addPointCloud(cloud_plane, "cloud_plane");
	viewer.spin();
	viewer.removePointCloud("cloud_plane");
	std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
	writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

	// Remove the planar inliers, extract the rest
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	// 此时 cloud_filtered2 已经是平面剔除后杯子与桌子部分的点云的
	viewer.addPointCloud(cloud_filtered2, "cloud_filtered2");
	viewer.spin();
	viewer.removePointCloud("cloud_filtered2");
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(cloud_normals);
	extract_normals.setIndices(inliers_plane);
	extract_normals.filter(*cloud_normals2);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.05);
	seg.setRadiusLimits(0, 0.1);
	seg.setInputCloud(cloud_filtered2);
	seg.setInputNormals(cloud_normals2);

	// Obtain the cylinder inliers and coefficients
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

	// Write the cylinder inliers to disk
	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_cylinder);

	viewer.addPointCloud(cloud_cylinder, "cloud_cylinder");
	viewer.spin();
	viewer.removePointCloud("cloud_cylinder");

	if (cloud_cylinder->points.empty())
		std::cerr << "Can't find the cylindrical component." << std::endl;
	else
	{
		std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
		writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
	}
	return (0);
	
}