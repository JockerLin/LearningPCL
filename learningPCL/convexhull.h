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
	pcl::io::loadPCDFile<pcl::PointXYZ>("mask/mask_error1.pcd", *cloud);
	//由于源数据降采样十倍导致 尺寸由（6000,3200）=>(600,320)
	int rows = 600, cols = 320;
	cv::Mat range_image = cv::Mat(rows, cols, CV_32FC1, cv::Scalar::all(0));
	cv::Mat range_image_normal = cv::Mat(rows, cols, CV_32FC1, cv::Scalar::all(0));
	cv::Mat range_image_uint;
	//todo:0828 归一化方法还是要修改，初始定义全0图像，后续归一化因素也会受到0的影响 done已修改
	//todo恢复成连续
	cout << (double)(cloud->points.size()) / (double)(rows * cols);
	//为什么背景的比例占这么大，有效点数据这么少 只有0.009xx
	float max_z, min_z;

	for (int i = 0; i < cloud->points.size(); i++) {
		
		pcl::PointXYZ pt = cloud->points.at(i);
		if (0 == i) {
			max_z = pt.z;
			min_z = pt.z;
		}
		else {
			if (pt.z > max_z) {
				max_z = pt.z;
			}
			if (pt.z < min_z) {
				min_z = pt.z;
			}
		}
		int x = pt.x * 40;
		int y = pt.y * 20;
		range_image.at<float>(x, y) = pt.z;
		//cout << x << " " << y << " " << pt.z << endl;
	}

	// 自定义归一方案能消除无像素区域的影响
	double normal_rate = (double)(255.0) / (max_z - min_z);
	for (int iRow = 0; iRow < rows; iRow = iRow + 1) {
		for (int jCol = 0; jCol < cols; jCol = jCol + 1) {
			double cur_z = range_image.at<float>(iRow, jCol);
			float normal_z_int = 0.0;
			if (cur_z != 0) {
				//不为零就是正常的数据
				float normal_z = (cur_z - min_z)*normal_rate;
				normal_z_int = normal_z;

			}
			range_image_normal.at<float>(iRow, jCol) = normal_z_int;
			//cout << "cur_z:" << cur_z <<", "<< "normal z:" << normal_z_int << endl;
		}
	}

	// cv归一方案
	//cv::normalize(range_image, range_image_normal, 255, 0.0, cv::NORM_MINMAX);

	range_image_normal.convertTo(range_image_uint, CV_8U);
	cv::imwrite("range_image1.jpg", range_image_uint);
	cv::namedWindow("range image", cv::WINDOW_NORMAL);
	cv::imshow("range image", range_image_uint);
	cv::waitKey(0);
	//cout << range_image_uint << endl;
	return 0;
}

int txt2RangeImage(string file, int rowsCount, char splitStr, int sampleRowsCols, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, bool saveFileFlag, string saveFile) {
	clock_t start_read_file = clock();
	bool save_model = true;

	ifstream fp(file);
	vector<vector<double>> user_arr;
	vector<vector<double>> range_image;

	string line;
	int rows_count = rowsCount;
	int cols_count = 3200;
	cv::Mat range_image_cv = cv::Mat(rows_count, cols_count, CV_32FC1, cv::Scalar::all(0));
	cv::Mat range_image_uint8 = cv::Mat(rows_count, cols_count, CV_8UC1, cv::Scalar::all(0));

	int add_number = 0;
	int max_x = 0, max_y = 0, max_z = 0;
	int min_x = 0, min_y = 0, min_z = 0;
	vector<double> height_data;
	double z_max = -0.001;
	double z_min = 0.001;
	int unvaliable_times = 0;
	bool init_min_max = false;

	for (int iRow = 0; iRow < rows_count; iRow = iRow + 1) {
		string line;
		getline(fp, line);
		//cout << "line before \n" << line << endl;

		// 删除\0
		string::iterator it;
		for (it = line.begin(); it != line.end(); it++)
		{
			if (*it == '\0')
			{
				line.erase(it); //STL erase函数
				if (it != line.begin()) {
					it--;
				}

			}
		}
		//cout << "line after \n" << line << endl;
		vector<double> data_line;
		vector<double> data_line_range;
		string number;
		istringstream readstr(line);
		if (0 == iRow % 200) {
			cout << "another 200 rows ==> " << iRow << endl;
		}

		for (int jCol = 0; jCol < cols_count; jCol = jCol + 1) {
			getline(readstr, number, splitStr);

			//float x = (float)jCol * 0.005;
			//float y = (float)iRow * 0.02;
			double z = (double)atoi(number.c_str())*0.00001;
			//极值初始化

			if ((false == init_min_max) && (z > -21.47483)) {
				z_max = z;
				z_min = z;
				init_min_max = true;
			}
			data_line.push_back(z);
			if (z > -21.47483 && init_min_max) {
				//z = z*0.00001;
				data_line_range.push_back(z);
				if (z < z_min) {
					z_min = z;
				}
				if (z >= z_max) {
					z_max = z;
				}
				//cout << "z:"<<z << endl;
			}
			else {
				//cout << "outside point:" << z << endl;
				z = 0;
				// 如果设置为0可能影响后续的归一化计算。。。。。。
				data_line_range.push_back(z);
				unvaliable_times++;
			}
		}
		user_arr.push_back(data_line);
		range_image.push_back(data_line_range);
	}
	double normal_rate = (double)(255.0) / (z_max - z_min);
	int all_datas_num = rows_count * cols_count;
	cout << "z max:" << z_max << ",z min:" << z_min << endl;
	cout << "unvaliable_times:" << unvaliable_times << endl;
	cout << "all_datas:" << all_datas_num << endl;
	cout << "rate:" << (double)unvaliable_times / (double)all_datas_num << endl;
	cout << "normal_rate:" << normal_rate << endl;

	for (int iRow = 0; iRow < rows_count; iRow = iRow + 1) {
		for (int jCol = 0; jCol < cols_count; jCol = jCol + 1) {
			double cur_z = range_image[iRow][jCol];
			float normal_z_int = 0.0;
			if (cur_z != 0) {
				//不为零就是正常的数据
				float normal_z = (cur_z - z_min)*normal_rate;
				normal_z_int = normal_z;

			}
			range_image_cv.at<float>(iRow, jCol) = normal_z_int;
			//cout << "cur_z:" << cur_z <<", "<< "normal z:" << normal_z_int << endl;
		}
	}

	//图像要uint8显示才不会出问题，出现全黑img可能是因为类型的问题。
	range_image_cv.convertTo(range_image_uint8, CV_8U);
	//归一化
	//cv::Mat mat_image(range_image);
	cv::namedWindow("range image", cv::WINDOW_NORMAL);
	cv::imshow("range image", range_image_uint8);
	cv::waitKey(0);
	cv::imwrite("range_image.jpg", range_image_uint8);
	//cout << range_image_uint8 << endl;

	fp.close();
	outCloud->width = (int)outCloud->points.size();
	outCloud->height = 1;
	return 0;

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
