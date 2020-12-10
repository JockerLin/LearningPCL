#pragma once
//#include <vtkAutoInit.h>
//#include <vtkRenderWindow.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// �������ͷ�ļ�
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "visualization.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT; // Ҳ����pcl::Normal,���޷���PCLVisualizer��ʾ��


static class SurfaceVector {
public:
	static void gengeratorVector();
	//static void demoVector();
};

//int demoVector(int argc, char** argv)
//{
//	// ��ȡ����
//	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
//	pcl::io::loadPCDFile(argv[1], *cloud);
//
//	// ���㷨����
//	pcl::NormalEstimation<PointT, PointNT> nest;
//	//nest.setRadiusSearch(0.01); // �������ʱ���������뾶�������ģ�ͷֱ��ʵı���
//	nest.setKSearch(50); // �������ʱ���õĵ���
//	nest.setInputCloud(cloud);
//	pcl::PointCloud<PointNT>::Ptr normals(new pcl::PointCloud<PointNT>);
//	nest.compute(*normals);
//
//	for (size_t i = 0; i < cloud->points.size(); ++i)
//	{	// ����ʱֻ�����˷�������û�н�ԭʼ������Ϣ������Ϊ����ʾ��Ҫ����ԭ��Ϣ
//		// Ҳ�������������������ӣ��磺pcl::concatenateFields
//		normals->points[i].x = cloud->points[i].x;
//		normals->points[i].y = cloud->points[i].y;
//		normals->points[i].z = cloud->points[i].z;
//	}
//
//	// ��ʾ
//	pcl::visualization::PCLVisualizer viewer;
//	viewer.addPointCloud(cloud, "cloud");
//	int level = 100; // ������������������ʾ��һ��
//	float scale = 0.01; // ����������
//	viewer.addPointCloudNormals<PointNT>(normals, level, scale, "normals");
//
//	viewer.spin();
//
//	system("pause");
//	return 0;
//}