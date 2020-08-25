#pragma once

#include <iostream>
#include <fstream>  
#include <sstream>
#include <vector>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>   // TicToc
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

using namespace std;

static class FileTools {
public:

	// 删除字符串中的相同字符
	static void deleteSameCharInString(string &operateStr, char deleteStr) {
		string::iterator it;
		for (it = operateStr.begin(); it != operateStr.end(); it++) {
			if (*it == deleteStr) {
				operateStr.erase(it);
				if (it != operateStr.begin()) {
					it--;
				}
			}
		}
	}

	// txt文本转点云数据
	static void txt2PCD(string file, int rowsCount, char splitStr, int sampleRowsCols, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, bool saveFileFlag, string saveFile) {
		clock_t start_read_file = clock();
		bool save_model = true;
		ifstream fp(file);
		vector<vector<double>> user_arr;
		vector<vector<double>> range_image;
		string line;
		int rows_count = rowsCount;
		int cols_count = 3200;
		int add_number = 0;
		int max_x = 0, max_y = 0, max_z = 0;
		int min_x = 0, min_y = 0, min_z = 0;
		vector<double> height_data;
		double z_max = 999999;
		double z_min = -99999;
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
				double z = (double)atoi(number.c_str());
				data_line.push_back(z);
				if (z > -2147483) {
					data_line_range.push_back(z);
					if (z < z_min) {
						z_min = z;
					}
					else if (z > z_max) {
						z_max = z;
					}
				}
				else {
					data_line_range.push_back(0);
				}
			}
			user_arr.push_back(data_line);
			range_image.push_back(data_line_range);
		}

		//归一化
		//cv::Mat mat_image(range_image);


		for (int iRow = 0; iRow < rows_count; iRow = iRow + sampleRowsCols) {
			for (int jCol = 0; jCol < cols_count; jCol = jCol + sampleRowsCols) {
				// 0.005 0.02
				double x = (double)jCol*0.005;
				double y = (double)iRow*0.0025;
				double z = user_arr[iRow][jCol] * 0.00001;
				// 去除背景信息
				// -2621440 2147483645 2147483645
				if (user_arr[iRow][jCol] > -2147483) {
					pcl::PointXYZ pt;
					// 移动是为了不都在原点
					pt.x = y + add_number;
					pt.y = x + add_number;
					pt.z = z;
					//cout << "z:" << z << endl;
					outCloud->points.push_back(pt);
				}
			}
		}

		fp.close();
		//data_file.close();
		outCloud->width = (int)outCloud->points.size();
		outCloud->height = 1;

		/*pcl::visualization::PCLVisualizer viewer;
		viewer.addPointCloud(outCloud, "cloud");
		viewer.spin();
		pcl::io::savePCDFile("phone1_add20.pcd", *outCloud);*/
		if (saveFileFlag) {
			pcl::io::savePCDFile(saveFile, *outCloud);
		}
		cout << "read file finished==>" << file << endl;
		cout << "Run time: " << (double)(clock() - start_read_file) << "mS" << endl;
	}

	static void readFileNoSample() {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		txt2PCD("C:/Users/suzhefeng/Downloads/1614_height/1614_height.txt", 6000, ',', 10, cloud, true, "08211614_height.pcd");
		//readCSVFile("G:/blazarlin/3dfiles/merge0.csv", 8000, ',', 10, cloud, true, "part0.pcd");
		pcl::visualization::PCLVisualizer viewer;
		viewer.addPointCloud(cloud, "cloud3");

		viewer.spin();
	}

	// txt文本转深度图
	static void txt2RangeImage(string file, int rowsCount, char splitStr, string saveFileName) {
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
		UINT8 replaceErrorValue = 255;
		// 读取文本值并解析
		for (int iRow = 0; iRow < rows_count; iRow = iRow + 1) {
			string line;
			getline(fp, line);
			// cout << "line before \n" << line << endl;

			// 删除\0
			deleteSameCharInString(line, '\0');
			
			vector<double> data_line;
			vector<double> data_line_range;
			string number;
			istringstream readstr(line);
			if (0 == iRow % 200) {
				cout << "another 200 rows ==> " << iRow << endl;
			}

			for (int jCol = 0; jCol < cols_count; jCol = jCol + 1) {
				getline(readstr, number, splitStr);
				double z = (double)atoi(number.c_str())*0.00001;

				//极值初始化 不能随意给0，会影响比例的计算
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
					z = replaceErrorValue;
					// 如果设置为0可能影响后续的归一化计算，现在不会了。
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

		//归一化，特殊的值就用255代替
		for (int iRow = 0; iRow < rows_count; iRow = iRow + 1) {
			for (int jCol = 0; jCol < cols_count; jCol = jCol + 1) {
				double cur_z = range_image[iRow][jCol];
				float normal_z_int = replaceErrorValue;
				if (cur_z != replaceErrorValue) {
					//不为零就是正常的数据
					float normal_z = (cur_z - z_min)*normal_rate;
					normal_z_int = normal_z;

				}
				range_image_cv.at<float>(iRow, jCol) = normal_z_int;
				//cout << "cur_z:" << cur_z <<", "<< "normal z:" << normal_z_int << endl;
			}
		}

		range_image_cv.convertTo(range_image_uint8, CV_8U);
		//归一化
		//cv::Mat mat_image(range_image);
		cv::namedWindow("range image", cv::WINDOW_NORMAL);
		cv::imshow("range image", range_image_uint8);
		cv::waitKey(0);
		cv::imwrite(saveFileName, range_image_uint8);
		fp.close();
	}
};