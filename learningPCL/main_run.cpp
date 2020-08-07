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


	// ���ƽ�����
	// PointCloudFilter::uniformSampling();

	// �˲�
	//PointCloudFilter::bilateralFilter();
	//PointCloudFilter::uniformSampling();

	// ���潵����
	//SurfaceVector::gengeratorVector();

	// ��ȡ�ؼ���
	//CalKeyPoints::HarrisDetector();
	//CalKeyPoints::siftDetector();
	//CalKeyPoints::ISSDetector();

	// ��������
	//FeatureDescribes::PFHOrSHOT();
	
	// example�������Ƽ��ϵĹؼ���ƥ�����transformation
	match2PointCloud();

	//showRotatePointCloud();
}