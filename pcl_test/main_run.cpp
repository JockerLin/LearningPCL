#include "pointcloudfilter.h"
#include "visualization.h"
#include "rangeimage.h"
#include "surfacevector.h"
#include "keypoints.h"

void main(int argc, char** argv) {
	//VisualLization::demoCallBack1();
	//lookRangeImage(argc, argv);

	// 点云降采样
	// PointCloudFilter::uniformSampling();

	// 表面降采样
	//SurfaceVector::gengeratorVector();

	// 获取关键点
	CalKeyPoints::HarrisDetector();
	
}