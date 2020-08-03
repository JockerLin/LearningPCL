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
	//pcl::PointCloud<pcl::PointXYZ> pointCloud;      //定义点云对象

	//for (float y = -0.5f; y <= 0.5f; y += 0.01f) {        //循环产生点数据

	//	for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
	//		pcl::PointXYZ point;
	//		point.x = 2.0f - y;
	//		point.y = y;
	//		point.z = z;
	//		pointCloud.points.push_back(point);          //循环添加点数据到点云对象
	//	}
	//}

	//pointCloud.width = (uint32_t)pointCloud.points.size();
	//pointCloud.height = 1;                            //设置点云对象的头信息


	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);

	// -----从创建的点云中获取深度图--//
	float angularResolution = (float)(1.0f * (M_PI / 180.0f)); // 按弧度1度
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f)); // 按弧度360.0度
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f)); // 按弧度180.0度
	// 传感器位置
	// 深度图采集的方向是从点云图的哪个角度？
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 2.0f, 0.0f);  //采集位置
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;     //深度图像遵循的坐标系统

	float noiseLevel = 0.00;
	float minRange = 0.0f;
	int borderSize = 1;

	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	/*
	 关于range_image.createFromPointCloud（）参数的解释 （涉及的角度都为弧度为单位） ：
	   point_cloud为创建深度图像所需要的点云
	  angular_resolution_x深度传感器X方向的角度分辨率
	  angular_resolution_y深度传感器Y方向的角度分辨率
	   pcl::deg2rad (360.0f)深度传感器的水平最大采样角度
	   pcl::deg2rad (180.0f)垂直最大采样角度
	   scene_sensor_pose设置的模拟传感器的位姿是一个仿射变换矩阵，默认为4*4的单位矩阵变换
	   coordinate_frame定义按照那种坐标系统的习惯  默认为CAMERA_FRAME
	   noise_level  获取深度图像深度时，邻近点对查询点距离值的影响水平
	   min_range 设置最小的获取距离，小于最小的获取距离的位置为传感器的盲区
	   border_size  设置获取深度图像边缘的宽度 默认为0
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

	setViewerPose2(viewer, range_image.getTransformationToWorldSystem());  //设置视点的位置
	viewer.initCameraParameters();
	//可视化深度图
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(range_image);

	viewer.spin();
}

int main() {
	//VisualLization::demoCallBack1();
	generatroRangeImage();
	return 0;
}