#pragma once
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>    //�������ͼ���ͷ�ļ�
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>   //���ͼ���ӻ���ͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>      //PCL���ӻ���ͷ�ļ�
#include <pcl/console/parse.h>

int lookRangeImage();
void showRangeImage();