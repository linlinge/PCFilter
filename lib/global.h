#pragma once
#include <ctime>
#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include "V3.hpp"
#include <Eigen/Dense>
#include <math.h>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include "FileExtend.h"
#include "Point.hpp"
#include <omp.h>
#include <unistd.h>

using namespace std;
extern vector<Point> ps_ply;
extern vector<Point> ps_xyz;
extern vector<Point> ps_confidence;
extern ifstream fin;
extern ofstream fout;
extern string line;
extern pcl::PointCloud<pcl::PointXYZRGBA>::Ptr P_opt;
extern pcl::PointCloud<pcl::PointXYZRGBA>::Ptr P_opt_mapped;
extern pcl::PointCloud<pcl::PointXYZRGBA>::Ptr P_true_registered;
extern pcl::PointCloud<pcl::PointXYZRGBA>::Ptr P_reproject;
extern pcl::PointCloud<pcl::PointXYZRGBA>::Ptr P_reproject_removal;
extern pcl::PointCloud<pcl::PointXYZRGBA>::Ptr P_error;
V3 get_color(float min,float max,float confidence);

