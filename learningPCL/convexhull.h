#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/io/file_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>


int PCL2RangeImage() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("mask/mask_error0.pcd", *cloud);
	
	cv::Mat range_image = cv::Mat(6000, 3200, CV_32FC1, cv::Scalar::all(0));
	cv::Mat range_image_normal = cv::Mat(6000, 3200, CV_32FC1, cv::Scalar::all(0));
	cv::Mat range_image_uint;

	//归一化 已经滤除了一些是否能恢复成连续
	for (int i = 0; i < cloud->points.size(); i++) {
		pcl::PointXYZ pt = cloud->points.at(i);
		int x = pt.x * 200;
		int y = pt.y * 400;
		range_image.at<float>(x, y) = pt.z;
		cout << pt.x << " " << pt.y << " " << pt.z << endl;
	}
	cv::normalize(range_image, range_image_normal, 255, 0.0, cv::NORM_MINMAX);
	range_image_normal.convertTo(range_image_uint, CV_8U);
	cv::namedWindow("range image", cv::WINDOW_NORMAL);
	cv::imshow("range image", range_image_uint);
	cv::waitKey(0);
	cout << range_image_uint << endl;
	
}

int calHull() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("mask/mask_error0.pcd", *cloud);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> nest;
	nest.setKSearch(20);
	nest.setInputCloud(cloud);
	pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
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
	int level = 1; // 多少条法向量集合显示成一条
	float scale = 0.05; // 法向量长度
	viewer.addPointCloudNormals<pcl::PointNormal>(normals, level, scale, "normals");

	viewer.spin();
	system("pause");
	return 0;
}

void detectHull() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("mask/mask_error0.pcd", *cloud);

	pcl::ConvexHull<pcl::PointXYZ> hull;
	hull.setInputCloud(cloud);
	hull.setDimension(3);

	std::vector<pcl::Vertices> polygons;
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
	hull.reconstruct(*surface_hull, polygons);

	cout << surface_hull->size() << endl;

	// ---------------------- Visualizer -------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	viewer->setBackgroundColor(255, 255, 255);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 255, 0);
	viewer->addPointCloud(cloud, color_handler, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handlerK(surface_hull, 255, 0, 0);
	viewer->addPointCloud(surface_hull, color_handlerK, "point");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "point");

	//viewer->addPolygon<pcl::PointXYZ>(surface_hull, 0, 0, 255, "polyline");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
}
