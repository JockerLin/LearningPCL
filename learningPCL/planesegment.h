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
#include <pcl/segmentation/extract_clusters.h>
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

int euclideanClusterExtraction() {
	// Read in the cloud data
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("examplemodel/table_scene_lms400.pcd", *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	// 降采样
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*
	pcl::visualization::PCLVisualizer viewer;

	// Create the segmentation object for the planar model and set all the parameters
	// 设置平面分割的参数
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	int i = 0, nr_points = (int)cloud_filtered->points.size();
	// 不停对cloud_filtered进行平面分割
	// 为什么是0.3 ???
	viewer.addPointCloud(cloud_filtered, "cloud_filtered");
	viewer.spin();
	viewer.removePointCloud("cloud_filtered");
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud_filtered = *cloud_f;
		viewer.addPointCloud(cloud_f, "cloud_f");
		viewer.spin();
		viewer.removePointCloud("cloud_f");
	}

	// Creating the KdTree object for the search method of the extraction
	// 聚类分割物体
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);
	
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

	int j = 0;
	int part_number = cluster_indices.size();
	// cluster_indices 是解析的个数
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_cluster_hsv(new pcl::PointCloud<pcl::PointXYZHSV>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
			/*pcl::PointXYZ pt = cloud_filtered->points[*pit];
			pcl::PointXYZHSV pt_hsv;
			pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZHSV>(pt, pt_hsv);*/
		}
			
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
		j++;
		double hsv_color_r = (double)255.0*(double)j / (double)part_number;
		cout << "r color:" << hsv_color_r << endl;
		int g = 255 * rand() / (RAND_MAX + 1.0f);
		int b = 255 * rand() / (RAND_MAX + 1.0f);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud_cluster, hsv_color_r, g, b);
		viewer.addPointCloud(cloud_cluster, color, ss.str());
	}
	viewer.spin();

	return (0);
}