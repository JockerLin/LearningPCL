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
#include "demofrominternet/matchrabbit.h"
#include "segmentation.h"
#include "convexhull.h"
#include "dataanalysis.h"


void showRotatePointCloud() {
	pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud_src);

	pcl::PointCloud<PointT>::Ptr cloud_tgt(new pcl::PointCloud<PointT>);
	VisualLization::rotatePointCloud(cloud_src, cloud_tgt);
	VisualLization::comPare2PointCloud(cloud_src, cloud_tgt);

}

void errorPtrWrite() {
	int *p;
	int var = 123;
	p = &var;
	cout << *p << endl;
}

void main(int argc, char** argv) {

	VisualLization::baseOperate();

	//�Զ������
	//PointCloudBaseOperate::generatePointCloudShow();
	//errorPtrWrite();
	//VisualLization::rotatePointCloud();

	// ���������
	// PointCloudBaseOperate::kdTreeFlann();

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
	// matchRabbitDemo();
	
	// getCorsbPoint();
	// saciaMatch();

	// �鿴�ֻ���ƴ���Ĳ���
	// VisualLization::watchPhone4Part();

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

	// �ָ�
	//Segmentation::planeSegment();
	//Segmentation::cylinderSegment();
	//Segmentation::euclideanClusterExtraction();
	//Segmentation::regionGrowingMask();
	//Segmentation::superVoxel();

	// ͹����� todo
	// detectHull();
	// calHull();
	// PCL2RangeImage();

	// showRangeImage();
	// �ļ�����ת��
	/*FileTools::txt2RangeImage(
		"C:/Users/suzhefeng/Downloads/0821maskdata/1620_height.txt",
		6000,
		',',
		"range_image0936.jpg");*/
	
}