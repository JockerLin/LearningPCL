//
// Created by pilcq on 2020/6/12.
//

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointRGB;


static class PointCloudBaseOperate {
public:
	
	// 示例1 生成点云并显示
	static int generatePointCloudShow() {
		std::cout << "Test PCL !" << std::endl;
		//定义点云
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		uint8_t r(255), g(15), b(15);

		//绘制一个椭圆柱体
		for (float z(-1.0); z <= 1.0; z += 0.05) {
			for (float angle(0.0); angle <= 360.0; angle += 5.0) {
				pcl::PointXYZRGB point;
				point.x = 0.5 * cosf(pcl::deg2rad(angle));
				point.y = sinf(pcl::deg2rad(angle));
				point.z = z;
				uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
				point.rgb = *reinterpret_cast<float*>(&rgb);
				point_cloud_ptr->points.push_back(point);
			}
			if (z < 0.0) {
				r -= 12;
				g += 12;
			}
			else {
				g -= 12;
				b += 12;
			}
		}
		// 定义point cloud的尺寸
		point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
		point_cloud_ptr->height = 1;

		// 显示点云
		pcl::visualization::CloudViewer viewer("test");
		viewer.showCloud(point_cloud_ptr);
		while (!viewer.wasStopped()) {};

		cout << "点云大小： " << point_cloud_ptr->size() << endl;

		// 保存点云
		pcl::io::savePCDFile("my_circle_building.pcd", *point_cloud_ptr);
		return 0;
	}

	// 读取点云文件
	static int readPointCloudFile() {

		//先定义一个
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

		//读取
		// "my_circle_building.pcd" "rabbit.pcd"
		if (pcl::io::loadPCDFile<PointT>("rabbit.pcd", *cloud) == -1) {
			PCL_ERROR("couldn't read file\n");
			return -1;
		}

		cout << "点云大小：" << cloud->size() << "," << cloud->width << "," << cloud->height << endl;
		showPointCloud(cloud);
		system("pause");
		return 0;
	}

	// 显示点云
	static int showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
		pcl::visualization::CloudViewer viewer("pcl");
		viewer.showCloud(pc);
		while (!viewer.wasStopped()) {};
		return 0;

	}

	// kdtree k近邻搜索 查询最近的k个点 邻域内的最近点
	static void kNeastSearch() {
		pcl::PointCloud<PointRGB>::Ptr cloud(new pcl::PointCloud<PointRGB>);
		pcl::io::loadPCDFile("rabbit.pcd", *cloud);

		//定义kdtree
		pcl::search::KdTree<PointRGB>::Ptr kdtree(new pcl::search::KdTree<PointRGB>);
		kdtree->setInputCloud(cloud);

		vector<int> indices, indices2;
		vector<float> distances, distances2;

		PointRGB point = cloud->points[0];
		cloud->points[0].g = 255;
		
		// 查询距point最近的k个点
		int k = 30;
		int size = kdtree->nearestKSearch(point, k, indices, distances);
		cout << "search point:" << size << endl;

		//更改临近点的颜色 可视化
		for (int i = 0; i < indices.size(); i++) {
			int index = indices[i];
			cloud->points[index].r = 255;
		}
		pcl::visualization::CloudViewer viewer("pcl");
		viewer.showCloud(cloud);
		while (!viewer.wasStopped()) {};


		// 查询point半径为radius邻域球内的点
		double radius = 0.5;
		size = kdtree->radiusSearch(point, radius, indices, distances);
		cout << "search point :" << size << std::endl;

		system("pause");

	}

	// kdtree flann 搜索最邻近点
	static void kdTreeFlann() {
		pcl::PointCloud<PointRGB>::Ptr cloud(new pcl::PointCloud<PointRGB>);
		pcl::io::loadPCDFile("rabbit.pcd", *cloud);

		//定义kdtree
		pcl::KdTreeFLANN<PointRGB>::Ptr kdtree(new pcl::KdTreeFLANN<PointRGB>);
		kdtree->setInputCloud(cloud);

		vector<int> indices, indices2;
		vector<float> distances, distances2;

		PointRGB point = cloud->points[0];
		cloud->points[0].g = 255;

		// 查询距point最近的k个点
		int k = 30;
		int size = kdtree->nearestKSearch(point, k, indices, distances);
		cout << "search point:" << size << endl;

		//更改临近点的颜色 可视化
		for (int i = 0; i < indices.size(); i++) {
			int index = indices[i];
			cloud->points[index].r = 255;
		}
		pcl::visualization::CloudViewer viewer("pcl");
		viewer.showCloud(cloud);
		while (!viewer.wasStopped()) {};


		// 查询point半径为radius邻域球内的点
		double radius = 0.5;
		size = kdtree->radiusSearch(point, radius, indices, distances);
		cout << "search point :" << size << std::endl;

		system("pause");
	}
};




int testConst() {
	//    testPcl();

//    const int p = 9;
//    const void * vp = &p;
//
//    const int *ptr;
//    *ptr = 10;
//    const int *ptr;
//    int val = 3;
//    ptr = &val;

//    int num=0;
//    int * const ptr=&num; //const指针必须初始化！且const指针的值不能修改
//    int * t = &num;
//    *t = 12;
//    cout<<*ptr<<endl;

//    const int num=10;
//    const int * ptr=&num;

//    const void * hm = 'jh';
//    const int * ptr;
//    int val = 3;
//    ptr = &val;
//    cout<<*ptr<<endl;
//
//    int *ptr1 = &val;
//    *ptr1=4;
//    cout<<*ptr<<endl;

//    int * const ptr 指针的地址是常量，指针指向的值是变量,const修饰ptr
//    const int * ptr 指针的地址是变量，指针指向的值是常量,const修饰*
	const int num = 0;
	int const * ptr = &num; //error! const int* -> int*
	cout << *ptr << endl;
	return 0;
}

void showRabit() {

}

class Apple {
public:
	static int i;

	Apple() {


	};
};

int Apple::i = 1;

int testStatic() {

	Apple obj1;
	Apple obj2;
	obj1.i = 2;
	obj2.i = 3;

	// prints value of i
	cout << obj1.i << " " << obj2.i;
	return 0;
}



int main2(int argc, char **argv) {
	PointCloudBaseOperate::kdTreeFlann();

	return 0;
}