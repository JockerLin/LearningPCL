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

	//自定义点云
	//PointCloudBaseOperate::generatePointCloudShow();
	//errorPtrWrite();
	//VisualLization::rotatePointCloud();

	// 最近邻搜索
	// PointCloudBaseOperate::kdTreeFlann();

	// 点云降采样
	// PointCloudFilter::uniformSampling();

	// 滤波
	// PointCloudFilter::bilateralFilter();
	// PointCloudFilter::uniformSampling();

	// 表面降采样
	//SurfaceVector::gengeratorVector();

	// 获取关键点
	//CalKeyPoints::HarrisDetector();
	//CalKeyPoints::siftDetector();
	//CalKeyPoints::ISSDetector();

	// 特征描述
	//FeatureDescribes::PFHOrSHOT();
	
	// 两个点云集合的关键点匹配算出transformation
	// match2PointCloud();
	// matchRabbitDemo();
	
	// getCorsbPoint();
	// saciaMatch();

	// 查看手机壳拼接四部分
	// VisualLization::watchPhone4Part();

	// showRotatePointCloud();
	//FeatureDescribes::ROPS();
	// lookRangeImage();
	// VisualLization::readCSVFile();

	// 测试icp
	// icpMatch();
	/*pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);
	pcl::PointCloud<PointT>::Ptr rotate(new pcl::PointCloud<PointT>);
	VisualLization::rotatePointCloud(cloud, rotate);
	pcl::io::savePCDFile("rabbit_rotate.pcd", *rotate);*/

	// 分割
	//Segmentation::planeSegment();
	//Segmentation::cylinderSegment();
	//Segmentation::euclideanClusterExtraction();
	//Segmentation::regionGrowingMask();
	//Segmentation::superVoxel();

	// 凸包检测 todo
	// detectHull();
	// calHull();
	// PCL2RangeImage();

	// showRangeImage();
	// 文件数据转换
	/*FileTools::txt2RangeImage(
		"C:/Users/suzhefeng/Downloads/0821maskdata/1620_height.txt",
		6000,
		',',
		"range_image0936.jpg");*/
	
}