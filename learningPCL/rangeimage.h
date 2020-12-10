#pragma once
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>    //关于深度图像的头文件
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>   //深度图可视化的头文件
#include <pcl/visualization/pcl_visualizer.h>      //PCL可视化的头文件
#include <pcl/console/parse.h>

int lookRangeImage();
void showRangeImage();