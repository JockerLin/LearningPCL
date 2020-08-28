#pragma once
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pointcloudfilter.h"
//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

static class Segmentation {
public:
	// 去除原始点云采集下的背景数据
	static int planeSegment() {
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
		cout << "coefficients size:" << coefficients->values.size() << endl;
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
	static int cylinderSegment() {
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

	// 根据欧氏距离分割
	static int euclideanClusterExtraction() {
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
		ec.setClusterTolerance(0.02); //0.02m=>2cm, 标准单位1.00m 
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
			double color_r = (double)255.0*(double)j / (double)part_number;
			cout << "r color:" << color_r << endl;
			int g = 255 * rand() / (RAND_MAX + 1.0f);
			int b = 255 * rand() / (RAND_MAX + 1.0f);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_cluster, color_r, g, b);
			viewer.addPointCloud(cloud_cluster, color, ss.str());
		}
		viewer.spin();

		return (0);
	}

	// 区域生长方法
	static int regionGrowing() {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile <pcl::PointXYZ>("examplemodel/region_growing_tutorial.pcd", *cloud) == -1)
		{
			std::cout << "Cloud reading failed." << std::endl;
			return (-1);
		}
		
		// 计算法向量
		pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod(tree);
		normal_estimator.setInputCloud(cloud);
		normal_estimator.setKSearch(50);
		normal_estimator.compute(*normals);
		
		//滤除Z数据
		pcl::IndicesPtr indices(new std::vector <int>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 1.0);
		pass.filter(*indices);

		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
		reg.setMinClusterSize(50);//最小集群点尺寸
		reg.setMaxClusterSize(1000000);//最大集群点尺寸
		reg.setSearchMethod(tree);
		reg.setNumberOfNeighbours(30);//领域点
		reg.setInputCloud(cloud);
		//reg.setIndices (indices);
		reg.setInputNormals(normals);
		reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);//法线与法线的角度差阈值
		reg.setCurvatureThreshold(1.0);//曲率法向量阈值

		std::vector <pcl::PointIndices> clusters;
		reg.extract(clusters);

		std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
		std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
		std::cout << "These are the indices of the points of the initial" <<
			std::endl << "cloud that belong to the first cluster:" << std::endl;
		//第0簇点集数据
		int counter = 0;
		while (counter < clusters[0].indices.size())
		{
			std::cout << clusters[0].indices[counter] << ", ";
			counter++;
			if (counter % 10 == 0)
				std::cout << std::endl;
		}
		std::cout << std::endl;

		pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
		pcl::visualization::PCLVisualizer viewer("Cluster viewer");
		viewer.addPointCloud(colored_cloud);
		//viewer.showCloud(colored_cloud);
		viewer.spin();
		/*while (!viewer.wasStopped())
		{
		}*/

		return (0);
	}

	static int regionGrowingMask() {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile <pcl::PointXYZ>("mask/mask_error0.pcd", *cloud) == -1)
		{
			std::cout << "Cloud reading failed." << std::endl;
			return (-1);
		}

		// 计算法向量
		pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod(tree);
		normal_estimator.setInputCloud(cloud);
		normal_estimator.setKSearch(50);
		normal_estimator.compute(*normals);

		//滤除Z数据
		pcl::IndicesPtr indices(new std::vector <int>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 1.0);
		pass.filter(*indices);

		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
		reg.setMinClusterSize(50);//最小集群点尺寸
		reg.setMaxClusterSize(1000000);//最大集群点尺寸
		reg.setSearchMethod(tree);
		reg.setNumberOfNeighbours(30);//领域点
		reg.setInputCloud(cloud);
		//reg.setIndices (indices);
		reg.setInputNormals(normals);
		reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);//法线与法线的角度差阈值
		reg.setCurvatureThreshold(5.0);//曲率法向量阈值

		std::vector <pcl::PointIndices> clusters;
		reg.extract(clusters);

		std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
		std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
		std::cout << "These are the indices of the points of the initial" <<
			std::endl << "cloud that belong to the first cluster:" << std::endl;
		//第0簇点集数据
		int counter = 0;
		while (counter < clusters[0].indices.size())
		{
			std::cout << clusters[0].indices[counter] << ", ";
			counter++;
			if (counter % 10 == 0)
				std::cout << std::endl;
		}
		std::cout << std::endl;

		pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
		pcl::visualization::PCLVisualizer viewer("Cluster viewer");
		viewer.addPointCloud(colored_cloud);
		//viewer.showCloud(colored_cloud);
		viewer.spin();
		/*while (!viewer.wasStopped())
		{
		}*/

		return (0);
	}

	//体素方法
	static int superVoxel() {
		/*if (argc < 2)
		{
			pcl::console::print_error("Syntax is: %s <pcd-file> \n "
				"--NT Dsables the single cloud transform \n"
				"-v <voxel resolution>\n-s <seed resolution>\n"
				"-c <color weight> \n-z <spatial weight> \n"
				"-n <normal_weight>\n", argv[0]);
			return (1);
		}*/


		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::console::print_highlight("Loading point cloud...\n");
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("mask/mask_error0.pcd", *cloud))
		{
			pcl::console::print_error("Error loading cloud file!\n");
			return (1);
		}

		bool disable_transform = true;
		float voxel_resolution = 0.008f;
		float seed_resolution = 0.1f;
		float color_importance = 0.2f;
		float spatial_importance = 0.4f;
		float normal_importance = 1.0f;

		//////////////////////////////  //////////////////////////////
		////// This is how to use supervoxels
		//////////////////////////////  //////////////////////////////

		pcl::SupervoxelClustering<pcl::PointXYZRGBA> super(voxel_resolution, seed_resolution);
		if (disable_transform)
			super.setUseSingleCameraTransform(false);
		super.setInputCloud(cloud);
		super.setColorImportance(color_importance);
		super.setSpatialImportance(spatial_importance);
		super.setNormalImportance(normal_importance);

		std::map <std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;

		pcl::console::print_highlight("Extracting supervoxels!\n");
		super.extract(supervoxel_clusters);
		pcl::console::print_info("Found %d supervoxels\n", supervoxel_clusters.size());

		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(0, 0, 0);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
		viewer->addPointCloud(voxel_centroid_cloud, "voxel centroids");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "voxel centroids");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "voxel centroids");

		pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud();
		viewer->addPointCloud(labeled_voxel_cloud, "labeled voxels");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "labeled voxels");

		pcl::PointCloud<pcl::PointNormal>::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(supervoxel_clusters);
		//We have this disabled so graph is easy to see, uncomment to see supervoxel normals
		//viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

		pcl::console::print_highlight("Getting supervoxel adjacency\n");
		std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
		super.getSupervoxelAdjacency(supervoxel_adjacency);
		//To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
		for (auto label_itr = supervoxel_adjacency.cbegin(); label_itr != supervoxel_adjacency.cend(); )
		{
			//First get the label
			std::uint32_t supervoxel_label = label_itr->first;
			//Now get the supervoxel corresponding to the label
			pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);

			//Now we need to iterate through the adjacent supervoxels and make a point cloud of them
			pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
			for (auto adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first; adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr)
			{
				pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr->second);
				adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
			}
			//Now we make a name for this polygon
			std::stringstream ss;
			ss << "supervoxel_" << supervoxel_label;
			//This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
			addSupervoxelConnectionsToViewer(supervoxel->centroid_, adjacent_supervoxel_centers, ss.str(), viewer);
			//Move iterator forward to next label
			label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
		}

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
		}
		return (0);
	}

	static void addSupervoxelConnectionsToViewer(pcl::PointXYZRGBA &supervoxel_center,
		pcl::PointCloud<pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
		std::string supervoxel_name,
		pcl::visualization::PCLVisualizer::Ptr & viewer) {

		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
		vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();

		//Iterate through all adjacent points, and add a center point to adjacent point pair
		for (auto adjacent_itr = adjacent_supervoxel_centers.begin(); adjacent_itr != adjacent_supervoxel_centers.end(); ++adjacent_itr)
		{
			points->InsertNextPoint(supervoxel_center.data);
			points->InsertNextPoint(adjacent_itr->data);
		}
		// Create a polydata to store everything in
		vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
		// Add the points to the dataset
		polyData->SetPoints(points);
		polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
		for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
			polyLine->GetPointIds()->SetId(i, i);
		cells->InsertNextCell(polyLine);
		// Add the lines to the dataset
		polyData->SetLines(cells);
		viewer->addModelFromPolyData(polyData, supervoxel_name);
	}
};