#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <time.h>
#include "visualization.h"

typedef pcl::PointXYZ PointT;

static class CalKeyPoints {
public:

	// ISS Key Points
	static void ISSDetector() {
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		//pcl::io::loadPCDFile("rabbit.pcd", *cloud);
		cloud = VisualLization::getPointCloud("rabbit.pcd");
		std::cout << "original cloud size : " << cloud->size() << std::endl;
		clock_t start = clock();
		// �ֱ���ԽС �ؼ���Խ��
		double resolution = 0.0005;

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

		pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
		iss_detector.setSearchMethod(tree);
		iss_detector.setSalientRadius(6 * resolution);
		iss_detector.setNonMaxRadius(4 * resolution);
		iss_detector.setThreshold21(0.975);
		iss_detector.setThreshold32(0.975);
		iss_detector.setMinNeighbors(5);
		iss_detector.setNumberOfThreads(4);
		iss_detector.setInputCloud(cloud);

		pcl::PointCloud<PointT>::Ptr keys(new pcl::PointCloud<PointT>);
		iss_detector.compute(*keys);
		clock_t end = clock();
		std::cout << "spend time:" << (double)(end - start) / CLOCKS_PER_SEC << std::endl;
		std::cout << "key points size : " << keys->size() << std::endl;

		VisualLization::comPare2PointCloud(cloud, keys);
		system("pause");
	}

	static void HarrisDetector() {
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		//pcl::io::loadPCDFile("rabbit.pcd", *cloud);
		cloud = VisualLization::getPointCloud("rabbit.pcd");
		std::cout << "original cloud size : " << cloud->size() << std::endl;
		clock_t start = clock();
		// ��������ģ�� 0.0005�ķֱ���35947��������221���ؼ���
		// �ֱ���ԽС �ؼ���Խ��
		double resolution = 0.0005;

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

		pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI> detector;
		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<pcl::PointXYZI>);
		detector.setNonMaxSupression(true);
		detector.setRadiusSearch(10 * resolution);
		detector.setThreshold(1E-6);
		detector.setSearchMethod(tree); // ��дҲ���ԣ�Ĭ�Ϲ���kdtree
		detector.setInputCloud(cloud);
		detector.compute(*keypoints_temp);
		pcl::console::print_highlight("Detected %d points !\n", keypoints_temp->size());
		pcl::PointCloud<PointT>::Ptr keys(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*keypoints_temp, *keys);

		VisualLization::comPare2PointCloud(cloud, keys);
	}
	//static void demoVector();
};


