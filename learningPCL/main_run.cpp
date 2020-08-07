#include "pointcloudfilter.h"
#include "visualization.h"
#include "rangeimage.h"
#include "surfacevector.h"
#include "keypoints.h"
#include "featuredescribes.h"
#include "matchdemo.h"

void showRotatePointCloud() {
	pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud_src);

	pcl::PointCloud<PointT>::Ptr cloud_tgt(new pcl::PointCloud<PointT>);
	VisualLization::rotatePointCloud(cloud_src, cloud_tgt);
	VisualLization::comPare2PointCloud(cloud_src, cloud_tgt);
}

void main(int argc, char** argv) {
	//VisualLization::demoCallBack1();
	//lookRangeImage(argc, argv);

	//VisualLization::rotatePointCloud();


	// 点云降采样
	// PointCloudFilter::uniformSampling();

	// 滤波
	//PointCloudFilter::bilateralFilter();
	//PointCloudFilter::uniformSampling();

	// 表面降采样
	//SurfaceVector::gengeratorVector();

	// 获取关键点
	//CalKeyPoints::HarrisDetector();
	//CalKeyPoints::siftDetector();
	//CalKeyPoints::ISSDetector();

	// 特征描述
	//FeatureDescribes::PFHOrSHOT();
	
	// example两个点云集合的关键点匹配算出transformation
	match2PointCloud();

	//showRotatePointCloud();
}