#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "visualization.h"
#include "featuredescribes.h"
#include "keypoints.h"
#include "visualization.h"
#include "try_icp.h"

/*
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
*/
//#include "resolution.h" // 用于计算模型分辨率
//#include "getTransformation.h" //计算旋转平移矩阵

typedef pcl::PointXYZ PointT;
typedef pcl::SHOT352 FeatureT;

// 获取Harris关键点
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

void getISSKeyPoints(const pcl::PointCloud<PointT>::Ptr &cloud, double resolution,
	pcl::PointCloud<PointT>::Ptr &keys) {

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
	iss_detector.setSearchMethod(tree);
	//iss_detector.setBorderRadius()
	iss_detector.setSalientRadius(6 * resolution);
	iss_detector.setNonMaxRadius(4 * resolution);
	iss_detector.setThreshold21(0.975);
	iss_detector.setThreshold32(0.975);
	iss_detector.setMinNeighbors(5);
	iss_detector.setNumberOfThreads(4);
	iss_detector.setInputCloud(cloud);
	iss_detector.compute(*keys);
	pcl::console::print_highlight("Detected %d points !\n", keys->size());
}

void getFeatures(const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<PointT>::Ptr &keys,
	double resolution, pcl::PointCloud<FeatureT>::Ptr features)
{
	// 法向量
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
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud_input_src(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("phone0_add0.pcd", *cloud_input_src);

	pcl::PointCloud<PointT>::Ptr cloud_input_tgt(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("phone2_add20.pcd", *cloud_input_tgt);
	//VisualLization::rotatePointCloud(cloud_src, cloud_tgt);

	// 选择ROI
	pcl::visualization::PCLVisualizer viewer_select_ROI;
	viewer_select_ROI.addPointCloud(cloud_input_src);
	struct VisualLization::callback_args3 cb_args;
	cb_args.orgin_points = cloud_input_src;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
	cb_args.chosed_points_3d = cloud_src;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer_select_ROI);
	viewer_select_ROI.registerAreaPickingCallback(VisualLization::ap_callback, (void*)&cb_args);
	viewer_select_ROI.spin();

	// 选择ROI
	pcl::visualization::PCLVisualizer viewer_select_ROI2;
	viewer_select_ROI2.addPointCloud(cloud_input_tgt);
	struct VisualLization::callback_args3 cb_args2;
	cb_args2.orgin_points = cloud_input_tgt;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);
	cb_args2.chosed_points_3d = cloud_tgt;
	cb_args2.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer_select_ROI2);
	viewer_select_ROI2.registerAreaPickingCallback(VisualLization::ap_callback, (void*)&cb_args2);
	viewer_select_ROI2.spin();

	// 计算模型分辨率
	double resolution = 0.05;

	// 提取关键点
	pcl::PointCloud<PointT>::Ptr keys_src(new pcl::PointCloud<pcl::PointXYZ>);
	getISSKeyPoints(cloud_src, resolution, keys_src);

	pcl::PointCloud<PointT>::Ptr keys_tgt(new pcl::PointCloud<pcl::PointXYZ>);
	getISSKeyPoints(cloud_tgt, resolution, keys_tgt);

	// 特征描述
	pcl::PointCloud<FeatureT>::Ptr features_src(new pcl::PointCloud<FeatureT>);
	getFeatures(cloud_src, keys_src, resolution, features_src);

	pcl::PointCloud<FeatureT>::Ptr features_tgt(new pcl::PointCloud<FeatureT>);
	getFeatures(cloud_tgt, keys_tgt, resolution, features_tgt);

	// 计算对应匹配关系
	pcl::registration::CorrespondenceEstimation<FeatureT, FeatureT> cor_est;
	cor_est.setInputCloud(features_src);
	cor_est.setInputTarget(features_tgt);
	boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);
	cor_est.determineReciprocalCorrespondences(*cor);
	std::cout << "compute Correspondences " << cor->size() << std::endl;

	/*
	// 也可以不用关键点对应关系直接获取旋转平移矩阵
	Eigen::Matrix4f transformation(Eigen::Matrix4f::Identity());
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
	svd.estimateRigidTransformation(*cloud_src, *cloud_tgt, transformation);
	*/

	// 显示
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud_src, "cloud_src"); // 显示点云
	viewer.addPointCloud(cloud_tgt, "cloud_tgt");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> red_src(keys_src, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red_tgt(keys_tgt, 255, 0, 0);
	viewer.addPointCloud(keys_src, red_src, "keys_src"); //显示关键点，红色，加粗
	viewer.addPointCloud(keys_tgt, red_tgt, "keys_tgt");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keys_src");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keys_tgt");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_for_trans(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_for_trans(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < cor->size(); ++i) // 显示关键点匹配关系
	{
		PointT temp1 = keys_src->points[cor->at(i).index_query];
		PointT temp2 = keys_tgt->points[cor->at(i).index_match];
		cloud_src_for_trans->points.push_back(temp1);
		cloud_tgt_for_trans->points.push_back(temp2);
		std::stringstream ss;
		ss << "line_" << i;
		viewer.addLine(temp1, temp2, ss.str());
	}

	// 计算旋转平移矩阵
	Eigen::Matrix4f transformation(Eigen::Matrix4f::Identity());
	//getTransformation(keys_src, keys_tgt, resolution, *cor, transformation);
	
	// 计算transform之前将两边点集数量整理为一致
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
	svd.estimateRigidTransformation(*cloud_src_for_trans, *cloud_tgt_for_trans, transformation);
	cout << "begin transformation" << endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud_src, *cloud_trans, transformation); // 将原点云旋转
	for (int i = 0; i < 4; i++) {
		cout << endl;
		for (int j = 0; j < 4; j++) {
			cout << transformation(i, j)<<" ";
		}
	}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_trans(cloud_trans, 0, 255, 0);
	viewer.addPointCloud(cloud_trans, green_trans, "cloud_trans");
	viewer.spin();

	// 加入icp,继续优化transformation
	ICPMatch::run(cloud_trans, cloud_tgt);

	// 0.302273 0.950977 0.0653723 75.2311
	// -0.952828 0.299467 0.0493814 35.3466
	//	0.0273838 - 0.0772152 0.996638 0.40969
	//	0 0 0 1

	

	pcl::visualization::PCLVisualizer viewer_finilly;
	// 对原始输入做T变换
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans_from_src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud_input_src, *cloud_trans_from_src, transformation); // 将原点云旋转
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> show_handler(cloud_trans_from_src, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> show_handler_tgt(cloud_input_tgt, 255, 0, 0);
	viewer_finilly.addPointCloud(cloud_trans_from_src, show_handler, "handler_trans");
	viewer_finilly.addPointCloud(cloud_input_tgt, show_handler_tgt, "tgt_handler");
	viewer_finilly.spin();
	system("pause");
	return 0;
}