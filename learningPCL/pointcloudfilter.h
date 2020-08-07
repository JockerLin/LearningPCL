#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/impl/bilateral.hpp>


typedef pcl::PointXYZ PointT;

class PointCloudFilter {
public:

	// 1-���ƽ�����
	static void uniformSampling() {
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile("rabbit.pcd", *cloud);
		pcl::console::print_highlight("load cloud!\n");
		std::cout << "original cloud size : " << cloud->size() << std::endl;

		// ʹ�����ػ�����(VoxelGrid)������
		pcl::VoxelGrid<PointT> grid;//�����˲�����
		const float leaf = 0.005f;
		grid.setLeafSize(leaf, leaf, leaf);//�����������
		grid.setInputCloud(cloud);//���õ���
		pcl::PointCloud<PointT>::Ptr voxelResult(new pcl::PointCloud<PointT>);
		grid.filter(*voxelResult);//ִ���˲�
		std::cout << "voxel downsample size :" << voxelResult->size() << std::endl;

		// uniformSampling������
		pcl::UniformSampling<PointT> uniform_sampling;
		uniform_sampling.setInputCloud(cloud);
		double radius = 0.005f;
		uniform_sampling.setRadiusSearch(radius);
		pcl::PointCloud<PointT>::Ptr uniformResult(new pcl::PointCloud<PointT>);
		uniform_sampling.filter(*uniformResult);
		cout << "UniformSampling size :" << uniformResult->size() << endl;

		// �������
		pcl::visualization::PCLVisualizer viewer;
		//pcl::visualization::PointCloudColorHandlerCustom<PointT> green(cloud, 0, 255, 0);
		viewer.addPointCloud(voxelResult, "cloud");
		viewer.spin();

		system("pause");
	}

	// 2-˫���˲�
	static void bilateralFilter() {
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::io::loadPCDFile("rabbit.pcd", *cloud);
		std::cout << "origin cloud size :" << cloud->size() << std::endl;

		pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
		pcl::BilateralFilter<pcl::PointXYZI> fbf;
		fbf.setInputCloud(cloud);
		fbf.setSearchMethod(tree1);
		fbf.setStdDev(0.1);
		fbf.setHalfSize(0.1);
		fbf.filter(*output);
		std::cout << "output size :" << output->size() << std::endl;

		/*pcl::visualization::PCLVisualizer viewer;
		viewer.addPointCloud(output);
		viewer.spin();*/

		system("pause");

	}
};