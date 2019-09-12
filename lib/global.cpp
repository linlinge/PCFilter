#include "global.h"
vector<Point> ps_ply;
vector<Point> ps_xyz;
vector<Point> ps_confidence;
ifstream fin;
ofstream fout;
string line;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr P_opt(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr P_opt_mapped(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr P_true_registered(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr P_reproject(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr P_reproject_removal(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr P_error(new pcl::PointCloud<pcl::PointXYZRGBA>);

V3 get_color(float min,float max,float confidence)
{
	V3 color_temp;
	float n=4.0; // 
	float step=(max-min)/n;
	float c=confidence-min;
	if(c<step)
	{
		color_temp.r=0;
		color_temp.g=c/step*255;
		color_temp.b=255;
	}
	else if(c<2*step)
	{
		color_temp.r=0;
		color_temp.g=255;
		color_temp.b=255-(c-step)/step*255;
	}
	else if(c<3*step)
	{
		color_temp.r=(c-2*step)/step*255;
		color_temp.g=255;
		color_temp.b=0;
	}
	else if(c<=4*step)
	{
		color_temp.r=255;
		color_temp.g=255-(c-3*step)/step*255;
		color_temp.b=0;
	}
	
	return color_temp;
}


char bar[102];
const char *lable = "|/-\\";
void progress(int i)
{	
	printf("[%-100s][%d%%][%c]\r", bar, i, lable[i%4]);
	fflush(stdout);
	bar[i] = '#';
	i++;
	bar[i] = 0;
	//usleep(100000);
}









