#pragma once

//�ο�
//https://github.com/MNewBie/PCL-Notes/blob/master/chapter11.md
//http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_4:_3D_object_recognition_(descriptors)
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot.h>

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNormal;
typedef pcl::PFHSignature125 FeaturePFH;
typedef pcl::SHOT352 FeatureSHOT;

static class FeatureDescribes {
public:
	static void PFHOrSHOT(string feaTypes="FPH") {
		// ��ȡ����
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile("rabbit.pcd", *cloud);

		// ��ȡ�ؼ��㣬Ҳ������֮ǰ�ᵽ�ķ�������
		pcl::PointCloud<PointT>::Ptr keys(new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile("rabbit_sift_kps.pcd", *keys);

		double resolution = 0.0005;

		// ������
		pcl::NormalEstimation<PointT, PointNormal> nest;
		// nest.setRadiusSearch(10 * resolution);
		nest.setKSearch(10);
		nest.setInputCloud(cloud);
		nest.setSearchSurface(cloud);
		pcl::PointCloud<PointNormal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		nest.compute(*normals);
		std::cout << "compute normal\n";

		// �ؼ������PFH������
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
		// keys ��649���ؼ���,ÿ������125��ֱ��ͼ��Ϣ��
		std::cout << "compute feature\n";

		system("pause");
	}
};