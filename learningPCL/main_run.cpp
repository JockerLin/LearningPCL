#include "pointcloudfilter.h"
#include "visualization.h"
#include "rangeimage.h"
#include "surfacevector.h"
#include "keypoints.h"
#include "featuredescribes.h"
#include "matchdemo.h"
#include "searchpoints.h"
#include "try_icp.h"
#include "sacia.h"


void showRotatePointCloud() {
	pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud_src);

	pcl::PointCloud<PointT>::Ptr cloud_tgt(new pcl::PointCloud<PointT>);
	VisualLization::rotatePointCloud(cloud_src, cloud_tgt);
	VisualLization::comPare2PointCloud(cloud_src, cloud_tgt);

}

void main(int argc, char** argv) {
	//�Զ������
	//PointCloudBaseOperate::generatePointCloudShow();
	
	//VisualLization::rotatePointCloud();

	// ���������
	//PointCloudBaseOperate::kdTreeFlann();

	// ���ƽ�����
	// PointCloudFilter::uniformSampling();

	// �˲�
	// PointCloudFilter::bilateralFilter();
	// PointCloudFilter::uniformSampling();

	// ���潵����
	//SurfaceVector::gengeratorVector();

	// ��ȡ�ؼ���
	//CalKeyPoints::HarrisDetector();
	//CalKeyPoints::siftDetector();
	//CalKeyPoints::ISSDetector();

	// ��������
	//FeatureDescribes::PFHOrSHOT();
	
	// �������Ƽ��ϵĹؼ���ƥ�����transformation
	// match2PointCloud();
	getCorsbPoint();
	// saciaMatch();

	// showRotatePointCloud();
	//FeatureDescribes::ROPS();
	// lookRangeImage();
	// VisualLization::readCSVFile();

	// ����icp
	// icpMatch();
	/*pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);
	pcl::PointCloud<PointT>::Ptr rotate(new pcl::PointCloud<PointT>);
	VisualLization::rotatePointCloud(cloud, rotate);
	pcl::io::savePCDFile("rabbit_rotate.pcd", *rotate);*/
}