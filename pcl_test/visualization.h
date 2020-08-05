#pragma once

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>   //深度图可视化的头文件

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointRGB;

void generatroRangeImage();

static class VisualLization {
public:

	// 1-点云可视化基本操作
	static void baseOperate() {

		// 读取
		pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile("my_circle_building.pcd", *cloud1);

		pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile("rabbit.pcd", *cloud2);

		// 定义对象
		pcl::visualization::PCLVisualizer viewer;
		//设置背景颜色，默认黑色
		viewer.setBackgroundColor(100, 100, 100); // rgb

		// --- 显示点云数据 ----
	// "cloud1" 为显示id，默认cloud,显示多个点云时用默认会报警告。
		pcl::visualization::PointCloudColorHandlerCustom<PointT> green(cloud2, 0, 255, 0); // rgb
		viewer.addPointCloud(cloud1, green, "cloud1");
		//viewer.addPointCloud(cloud1, "cloud1");


		pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud2, 255, 0, 0); // rgb
		// 将点云设置颜色，默认白色
		viewer.addPointCloud(cloud2, red, "cloud2");

		// 将两个点连线
		PointT temp1 = cloud1->points[0];
		PointT temp2 = cloud2->points[1];
		viewer.addLine(temp1, temp2, "line0");
		// 同样可以设置线的颜色，
		//viewer.addLine(temp1, temp2, 255，0，0， "line0");

		// --- 显示网格数据 ---
		/*pcl::PolygonMesh mesh;
		pcl::io::loadPLYFile("read.ply", mesh);

		viewer.addPolygonMesh(mesh);*/

		// 1. 阻塞式
		//viewer.spin();

		// 2. 非阻塞式
		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			// 可添加其他操作
		}
		system("pause");

	}

	// 2-回调函数交互，键盘显示点云显示隐藏
	struct callback_args1 {
		bool *isShow;
		pcl::PointCloud<pcl::PointXYZ>::Ptr origin_points;
		pcl::visualization::PCLVisualizer::Ptr viewerPtr;
	};
	static void kb_callback(const pcl::visualization::KeyboardEvent& event, void* args) {
		if (event.keyDown() && event.getKeyCode() == 'a') {
			cout << "a has pressed" << endl;
			struct callback_args1* data = (struct callback_args1 *)args;
			if (*(data->isShow)) {
				data->viewerPtr->removePointCloud("cloud");
				*(data->isShow) = false;
				cout << "remove" << endl;
			}
			else {
				data->viewerPtr->addPointCloud(data->origin_points, "cloud");
				*(data->isShow) = true;
				cout << "add" << endl;
			}
		}
	}
	static void demoCallBack1() {
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile("rabbit.pcd", *cloud);
		pcl::console::print_highlight("load cloud!\n");

		// 定义对象
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
		viewer->addPointCloud(cloud, "cloud");

		// 设定参数
		bool isShow = true;
		struct callback_args1 kb_args;
		kb_args.isShow = &isShow;
		kb_args.origin_points = cloud;
		kb_args.viewerPtr = viewer;

		viewer->registerKeyboardCallback(kb_callback, (void*)&kb_args);
		viewer->spin();

	}

	//3-单点选择 control+shift+click 选择点
	//but 为什么是control+shift 内部定义没找到
	struct callback_args2 {
		pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d;
		pcl::visualization::PCLVisualizer::Ptr viewerPtr;
	};
	static void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args) {
		struct callback_args2* data = (struct callback_args2 *)args;

		if (event.getPointIndex() == -1) {
			return;
		}
		int index = event.getPointIndex();
		cout << "index: " << index << endl;
		pcl::PointXYZ current_point;
		//获取点击点的坐标
		event.getPoint(current_point.x, current_point.y, current_point.z);
		data->clicked_points_3d->points.push_back(current_point);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(data->clicked_points_3d, 255, 0, 0);
		data->viewerPtr->removePointCloud("clicked_points");
		data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
		data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
		cout << current_point.x << " " << current_point.y << " " << current_point.z << " " << endl;

	}
	static void demoCallBack2() {
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile("rabbit.pcd", *cloud);
		pcl::console::print_highlight("load cloud!\n");

		// 定义对象
		pcl::visualization::PCLVisualizer viewer;
		pcl::visualization::PointCloudColorHandlerCustom<PointT> green(cloud, 0, 255, 0);
		viewer.addPointCloud(cloud, "cloud");

		struct callback_args2 cb_args;
		pcl::PointCloud<PointT>::Ptr clicked_points_3d(new pcl::PointCloud<PointT>);
		cb_args.clicked_points_3d = clicked_points_3d;
		cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
		viewer.registerPointPickingCallback(pp_callback, (void*)&cb_args);
		pcl::console::print_highlight("Shift+click on three floor points, then press 'Q'... \n");

		viewer.spin();
		system("pause");
	}

	// 4-点云区域选择  'x'进入选择模式
	struct callback_args3 {
		// structure used to pass arguments to the callback function
		pcl::PointCloud<pcl::PointXYZ>::Ptr orgin_points;
		pcl::PointCloud<pcl::PointXYZ>::Ptr chosed_points_3d;
		pcl::visualization::PCLVisualizer::Ptr viewerPtr;
	};
	static void ap_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
	{
		struct callback_args3* data = (struct callback_args3 *)args;
		std::vector<int> indiecs;

		if (!event.getPointsIndices(indiecs))
			return;
		for (int i = 0; i < indiecs.size(); ++i)
		{
			data->chosed_points_3d->push_back(data->orgin_points->points[indiecs[i]]);
		}

		// Draw clicked points in red:
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(data->chosed_points_3d, 255, 0, 0);
		data->viewerPtr->removePointCloud("chosed_points");
		data->viewerPtr->addPointCloud(data->chosed_points_3d, red, "chosed_points");
		data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "chosed_points");
		std::cout << "selected " << indiecs.size() << " points , now sum is " << data->chosed_points_3d->size() << std::endl;
	}
	static void demoCallBack3() {
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile("rabbit.pcd", *cloud);
		pcl::console::print_highlight("load cloud!\n");

		// 定义对象
		pcl::visualization::PCLVisualizer viewer;
		pcl::visualization::PointCloudColorHandlerCustom<PointT> green(cloud, 0, 255, 0);
		viewer.addPointCloud(cloud, green, "cloud");

		struct callback_args3 cb_args;
		cb_args.orgin_points = cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr chosed_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
		cb_args.chosed_points_3d = chosed_points_3d;
		cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
		viewer.registerAreaPickingCallback(ap_callback, (void*)&cb_args);
		pcl::console::print_highlight("press x enter slected model, then press 'qQ'...\n");

		// Spin until 'Q' is pressed:
		viewer.spin();

		// 单独显示选择的点
		pcl::visualization::PCLVisualizer viewer2;
		viewer2.addPointCloud(cb_args.chosed_points_3d, "cloud choose");
		viewer2.spin();

		system("pause");
	}

	// 5-显示界面的区域分割 同一viewer显示两个点云 但是会一起旋转放大
	static void demoCallBack4() {
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile("rabbit.pcd", *cloud);
		pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile("my_circle_building.pcd", *cloud2);
		pcl::console::print_highlight("load cloud!\n");

		// 定义对象
		pcl::visualization::PCLVisualizer viewer;

		int v1(1); // viewport
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.setBackgroundColor(255, 0, 0, v1);
		viewer.addPointCloud(cloud, "cloud1", v1);;

		int v2(2);// viewport
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v1);
		viewer.setBackgroundColor(0, 255, 0, v2);
		viewer.addPointCloud(cloud2, "cloud2", v2);;

		viewer.spin();

		system("pause");
	}
};
