#include "pointcloudfilter.h"
#include "visualization.h"
#include "rangeimage.h"
#include "surfacevector.h"
#include "keypoints.h"

void main(int argc, char** argv) {
	//VisualLization::demoCallBack1();
	//lookRangeImage(argc, argv);

	// ���ƽ�����
	// PointCloudFilter::uniformSampling();

	// ���潵����
	//SurfaceVector::gengeratorVector();

	// ��ȡ�ؼ���
	CalKeyPoints::HarrisDetector();
	
}