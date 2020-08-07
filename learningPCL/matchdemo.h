#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// �������ͷ�ļ�
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "visualization.h"
#include "featuredescribes.h"
#include "keypoints.h"
/*
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
*/
//#include "resolution.h" // ���ڼ���ģ�ͷֱ���
//#include "getTransformation.h" //������תƽ�ƾ���

typedef pcl::PointXYZ PointT;
typedef pcl::SHOT352 FeatureT;

// ��ȡHarris�ؼ���
void getHarrisKeyPoints(const pcl::PointCloud<PointT>::Ptr &cloud, double resolution,
	pcl::PointCloud<PointT>::Ptr &keys)
{
	pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI> detector;
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<pcl::PointXYZI>);
	detector.setNonMaxSupression(true);
	detector.setRadiusSearch(10 * resolution);
	detector.setThreshold(1E-6);
	detector.setInputCloud(cloud);
	detector.compute(*keypoints_temp);
	pcl::console::print_highlight("Detected %d points !\n", keypoints_temp->size());
	pcl::copyPointCloud(*keypoints_temp, *keys);
}

void getFeatures(const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<PointT>::Ptr &keys,
	double resolution, pcl::PointCloud<FeatureT>::Ptr features)
{
	// ������
	pcl::NormalEstimation<PointT, pcl::Normal> nest;
	nest.setKSearch(10);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
	nest.setInputCloud(cloud);
	nest.setSearchSurface(cloud);
	nest.compute(*cloud_normal);
	std::cout << "compute normal\n";

	pcl::SHOTEstimation<PointT, pcl::Normal, FeatureT> shot;
	shot.setRadiusSearch(18 * resolution);
	shot.setInputCloud(keys);
	shot.setSearchSurface(cloud);
	shot.setInputNormals(cloud_normal);
	shot.compute(*features);
	std::cout << "compute feature\n";
}

int match2PointCloud()
{
	// ��ȡ����
	pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud_src);

	pcl::PointCloud<PointT>::Ptr cloud_tgt(new pcl::PointCloud<PointT>);
	VisualLization::rotatePointCloud(cloud_src, cloud_tgt);

	// ����ģ�ͷֱ���
	double resolution = 0.0005;

	// ��ȡ�ؼ���
	pcl::PointCloud<PointT>::Ptr keys_src(new pcl::PointCloud<pcl::PointXYZ>);
	getHarrisKeyPoints(cloud_src, resolution, keys_src);

	pcl::PointCloud<PointT>::Ptr keys_tgt(new pcl::PointCloud<pcl::PointXYZ>);
	getHarrisKeyPoints(cloud_tgt, resolution, keys_tgt);

	// ��������
	pcl::PointCloud<FeatureT>::Ptr features_src(new pcl::PointCloud<FeatureT>);
	getFeatures(cloud_src, keys_src, resolution, features_src);

	pcl::PointCloud<FeatureT>::Ptr features_tgt(new pcl::PointCloud<FeatureT>);
	getFeatures(cloud_tgt, keys_tgt, resolution, features_tgt);

	// �����Ӧƥ���ϵ
	pcl::registration::CorrespondenceEstimation<FeatureT, FeatureT> cor_est;
	cor_est.setInputCloud(features_src);
	cor_est.setInputTarget(features_tgt);
	boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);
	cor_est.determineReciprocalCorrespondences(*cor);
	std::cout << "compute Correspondences " << cor->size() << std::endl;

	// ������תƽ�ƾ���
	Eigen::Matrix4f transformation(Eigen::Matrix4f::Identity());
	//getTransformation(keys_src, keys_tgt, resolution, *cor, transformation);
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
	svd.estimateRigidTransformation(*cloud_src, *cloud_tgt, transformation);

	/*
	// Ҳ���Բ��ùؼ����Ӧ��ϵֱ�ӻ�ȡ��תƽ�ƾ���
	Eigen::Matrix4f transformation(Eigen::Matrix4f::Identity());
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
	svd.estimateRigidTransformation(*cloud_src, *cloud_tgt, transformation);
	*/

	// ��ʾ
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud_src, "cloud_src"); // ��ʾ����
	viewer.addPointCloud(cloud_tgt, "cloud_tgt");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> red_src(keys_src, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red_tgt(keys_tgt, 255, 0, 0);
	viewer.addPointCloud(keys_src, red_src, "keys_src"); //��ʾ�ؼ��㣬��ɫ���Ӵ�
	viewer.addPointCloud(keys_tgt, red_tgt, "keys_tgt");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keys_src");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keys_tgt");

	for (size_t i = 0; i < cor->size(); ++i) // ��ʾ�ؼ���ƥ���ϵ
	{
		PointT temp1 = keys_src->points[cor->at(i).index_query];
		PointT temp2 = keys_tgt->points[cor->at(i).index_match];
		std::stringstream ss;
		ss << "line_" << i;
		viewer.addLine(temp1, temp2, ss.str());
	}

	pcl::PointCloud<PointT>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud_src, *cloud_trans, transformation); // ��ԭ������ת
	for (int i = 0; i < 4; i++) {
		cout << endl;
		for (int j = 0; j < 4; j++) {
			cout << transformation(i, j)<<" ";
		}
	}

	pcl::visualization::PointCloudColorHandlerCustom<PointT> green_trans(cloud_trans, 0, 255, 0);
	viewer.addPointCloud(cloud_trans, green_trans, "cloud_trans");

	viewer.spin();

	system("pause");
	return 0;
}