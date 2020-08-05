#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>   //���ͼ���ӻ���ͷ�ļ�
#include "visualization.h"


void
setViewerPose2(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0], up_vector[1], up_vector[2]);
}


void generatroRangeImage() {
	//pcl::PointCloud<pcl::PointXYZ> pointCloud;      //������ƶ���

	//for (float y = -0.5f; y <= 0.5f; y += 0.01f) {        //ѭ������������

	//	for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
	//		pcl::PointXYZ point;
	//		point.x = 2.0f - y;
	//		point.y = y;
	//		point.z = z;
	//		pointCloud.points.push_back(point);          //ѭ����ӵ����ݵ����ƶ���
	//	}
	//}

	//pointCloud.width = (uint32_t)pointCloud.points.size();
	//pointCloud.height = 1;                            //���õ��ƶ����ͷ��Ϣ


	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);

	// -----�Ӵ����ĵ����л�ȡ���ͼ--//
	float angularResolution = (float)(1.0f * (M_PI / 180.0f)); // ������1��
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f)); // ������360.0��
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f)); // ������180.0��
	// ������λ��
	// ���ͼ�ɼ��ķ����Ǵӵ���ͼ���ĸ��Ƕȣ�
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 2.0f, 0.0f);  //�ɼ�λ��
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;     //���ͼ����ѭ������ϵͳ

	float noiseLevel = 0.00;
	float minRange = 0.0f;
	int borderSize = 1;

	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	/*
	 ����range_image.createFromPointCloud���������Ľ��� ���漰�ĽǶȶ�Ϊ����Ϊ��λ�� ��
	   point_cloudΪ�������ͼ������Ҫ�ĵ���
	  angular_resolution_x��ȴ�����X����ĽǶȷֱ���
	  angular_resolution_y��ȴ�����Y����ĽǶȷֱ���
	   pcl::deg2rad (360.0f)��ȴ�������ˮƽ�������Ƕ�
	   pcl::deg2rad (180.0f)��ֱ�������Ƕ�
	   scene_sensor_pose���õ�ģ�⴫������λ����һ������任����Ĭ��Ϊ4*4�ĵ�λ����任
	   coordinate_frame���尴����������ϵͳ��ϰ��  Ĭ��ΪCAMERA_FRAME
	   noise_level  ��ȡ���ͼ�����ʱ���ڽ���Բ�ѯ�����ֵ��Ӱ��ˮƽ
	   min_range ������С�Ļ�ȡ���룬С����С�Ļ�ȡ�����λ��Ϊ��������ä��
	   border_size  ���û�ȡ���ͼ���Ե�Ŀ�� Ĭ��Ϊ0
	*/
	range_image.createFromPointCloud(
		*cloud,
		angularResolution,
		angularResolution,
		pcl::deg2rad(360.0f),
		pcl::deg2rad(180.0f),
		sensorPose,
		coordinate_frame,
		noiseLevel,
		minRange,
		borderSize);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcForShow(cloud);
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(pcForShow, "pc");
	viewer.setCameraPosition(0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

	setViewerPose2(viewer, range_image.getTransformationToWorldSystem());  //�����ӵ��λ��
	viewer.initCameraParameters();
	//���ӻ����ͼ
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(range_image);

	viewer.spin();
}

//int main() {
//	VisualLization::demoCallBack1();
//	//generatroRangeImage();
//	return 0;
//}