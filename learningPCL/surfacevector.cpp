#include "surfacevector.h"
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//VTK_MODULE_INIT(vtkRenderingOpenGL)
//VTK_MODULE_INIT(vtkInteractionStyle)


void SurfaceVector::gengeratorVector() {
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	cloud = VisualLization::getPointCloud("rabbit.pcd");

	// ��ȡ����
	//pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	//pcl::io::loadPCDFile("rabbit.pcd", *cloud);

	// ���㷨����
	pcl::NormalEstimation<PointT, PointNT> nest;
	//nest.setRadiusSearch(0.01); // �������ʱ���������뾶�������ģ�ͷֱ��ʵı���
	nest.setKSearch(20); // �������ʱ���õĵ���
	nest.setInputCloud(cloud);
	pcl::PointCloud<PointNT>::Ptr normals(new pcl::PointCloud<PointNT>);
	nest.compute(*normals);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{	// ����ʱֻ�����˷�������û�н�ԭʼ������Ϣ������Ϊ����ʾ��Ҫ����ԭ��Ϣ
		// Ҳ�������������������ӣ��磺pcl::concatenateFields
		normals->points[i].x = cloud->points[i].x;
		normals->points[i].y = cloud->points[i].y;
		normals->points[i].z = cloud->points[i].z;
	}

	// ��ʾ
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");
	int level = 5; // ������������������ʾ��һ��
	float scale = 0.002; // ����������
	viewer.addPointCloudNormals<PointNT>(normals, level, scale, "normals");

	viewer.spin();

	system("pause");
}

