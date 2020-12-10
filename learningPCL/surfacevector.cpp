#include "surfacevector.h"
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//VTK_MODULE_INIT(vtkRenderingOpenGL)
//VTK_MODULE_INIT(vtkInteractionStyle)


void SurfaceVector::gengeratorVector() {
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	cloud = VisualLization::getPointCloud("rabbit.pcd");

	// 读取点云
	//pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	//pcl::io::loadPCDFile("rabbit.pcd", *cloud);

	// 计算法向量
	pcl::NormalEstimation<PointT, PointNT> nest;
	//nest.setRadiusSearch(0.01); // 设置拟合时邻域搜索半径，最好用模型分辨率的倍数
	nest.setKSearch(20); // 设置拟合时采用的点数
	nest.setInputCloud(cloud);
	pcl::PointCloud<PointNT>::Ptr normals(new pcl::PointCloud<PointNT>);
	nest.compute(*normals);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{	// 生成时只生成了法向量，没有将原始点云信息拷贝，为了显示需要复制原信息
		// 也可用其他方法进行连接，如：pcl::concatenateFields
		normals->points[i].x = cloud->points[i].x;
		normals->points[i].y = cloud->points[i].y;
		normals->points[i].z = cloud->points[i].z;
	}

	// 显示
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");
	int level = 5; // 多少条法向量集合显示成一条
	float scale = 0.002; // 法向量长度
	viewer.addPointCloudNormals<PointNT>(normals, level, scale, "normals");

	viewer.spin();

	system("pause");
}

