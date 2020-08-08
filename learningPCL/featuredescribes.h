#pragma once

//参考
//https://github.com/MNewBie/PCL-Notes/blob/master/chapter11.md
//http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_4:_3D_object_recognition_(descriptors)
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot.h>
#include <pcl/features/rops_estimation.h>
#include <pcl/surface/gp3.h>


typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNormal;
typedef pcl::PFHSignature125 FeaturePFH;
typedef pcl::SHOT352 FeatureSHOT;

// 点云文件使用 "rabbit.pcd"
// 关键点文件 "rabbit_sift_kps.pcd"

static class FeatureDescribes {
public:
	static void PFHOrSHOT(string feaTypes="FPH") {
		// 读取点云
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile("rabbit.pcd", *cloud);

		// 读取关键点，也可以用之前提到的方法计算
		pcl::PointCloud<PointT>::Ptr keys(new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile("rabbit_sift_kps.pcd", *keys);

		double resolution = 0.0005;

		// 法向量
		pcl::NormalEstimation<PointT, PointNormal> nest;
		// nest.setRadiusSearch(10 * resolution);
		nest.setKSearch(10);
		nest.setInputCloud(cloud);
		nest.setSearchSurface(cloud);
		pcl::PointCloud<PointNormal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		nest.compute(*normals);
		std::cout << "compute normal\n";

		// 关键点计算PFH描述子
		// PFH
		//if ("FPH" == feaTypes) {
		//	pcl::PointCloud<FeaturePFH>::Ptr features(new pcl::PointCloud<FeaturePFH>);
		//	pcl::PFHEstimation<PointT, PointNormal, FeaturePFH> describes;
		//}
		//else if ("SHOT" == feaTypes) {
		//	//// SHOT
		//	pcl::PointCloud<FeatureSHOT>::Ptr features(new pcl::PointCloud<FeatureSHOT>);
		//	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> describes;
		//}
		//else {
		//	throw exception("gg");
		//}

		pcl::PointCloud<FeaturePFH>::Ptr features(new pcl::PointCloud<FeaturePFH>);
		pcl::PFHEstimation<PointT, PointNormal, FeaturePFH> describes;
		// SHOT
		//pcl::PointCloud<FeatureSHOT>::Ptr features(new pcl::PointCloud<FeatureSHOT>);
		//pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> describes;

		describes.setRadiusSearch(18 * resolution);
		describes.setSearchSurface(cloud);
		describes.setInputCloud(keys);
		describes.setInputNormals(normals);
		describes.compute(*features);
		// keys 有649个关键点,每个点有125个直方图信息。
		std::cout << "compute feature\n";

		system("pause");
	}

	static void ROPS() {
		// 加载点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile("rabbit.pcd", *cloud);

		// 加载关键点
		pcl::PointCloud<pcl::PointXYZ>::Ptr key_points(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile("rabbit_sift_kps.pcd", *key_points);

		// 计算法向量
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud);
		n.setInputCloud(cloud);
		n.setSearchMethod(tree);
		n.setKSearch(20);
		n.compute(*normals);

		// 连接数据
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
		//* cloud_with_normals = cloud + normals

		// ---- rops基于网格，所以要先将pcd点云数据重建网格 ---

		// Create search tree*
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud(cloud_with_normals);

		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;// Initialize objects
		pcl::PolygonMesh triangles;
		// Set the maximum distance between connected points (maximum edge length)
		gp3.setSearchRadius(0.025);
		gp3.setMu(2.5); // Set typical values for the parameters
		gp3.setMaximumNearestNeighbors(100);
		gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
		gp3.setMinimumAngle(M_PI / 18); // 10 degrees
		gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
		gp3.setNormalConsistency(false);
		gp3.setInputCloud(cloud_with_normals);
		gp3.setSearchMethod(tree2);
		gp3.reconstruct(triangles);	// Get result

		// ----- rops 描述-------
		// 由于pcl_1.8.0中rops还没有定义好的结构，所以采用pcl::Histogram存储描述子
		pcl::ROPSEstimation<pcl::PointXYZ, pcl::Histogram<135>> rops;
		rops.setInputCloud(key_points);
		rops.setSearchSurface(cloud);
		rops.setNumberOfPartitionBins(5);
		rops.setNumberOfRotations(3);
		rops.setRadiusSearch(0.01);
		rops.setSupportRadius(0.01);
		rops.setTriangles(triangles.polygons);
		rops.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree < pcl::PointXYZ>));
		//feature size = number_of_rotations * number_of_axis_to_rotate_around * number_of_projections * number_of_central_moments
		//unsigned int feature_size = number_of_rotations_ * 3 * 3 * 5; //计算出135
		pcl::PointCloud<pcl::Histogram<135>> description;
		rops.compute(description);  // 结果计算的是描述子。。需传入inputcloud和surface
		std::cout << "size is " << description.points.size() << std::endl;
		//pcl::io::savePCDFile("rops_des.pcd", description); // 此句出错！！pcl::Histogram没有对应的保存方法

		system("pause");
	}

};