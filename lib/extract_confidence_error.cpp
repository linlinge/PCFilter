#include "extract_confidence_error.h"
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include "V2.hpp"
vector<V2> error_confidence;

void extract_error(string P_opt_mapped_filename, string P_truth_registered_filename)
{
	CloudCompare cc;		
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);	
	pcl::io::loadPLYFile<pcl::PointXYZRGBA>(P_opt_mapped_filename,*cloud1);			
	pcl::io::loadPLYFile<pcl::PointXYZRGBA>(P_truth_registered_filename,*cloud2);						
	cc.find_correspondance(*cloud1,*cloud2);
	
	fout.open("error.txt");
	if(fout.is_open()==false)
	{
		cout<<"error.txt output file Error!"<<endl;
	}
	for(int i=0;i<cc.correspondance_.size();i++)
	{
		pcl::PointXYZRGBA p_temp;
		V3 color_temp=get_color(cc.min_dist_,cc.max_dist_,cc.correspondance_[i].dist_);
		p_temp.x=cloud1->points[i].x;
		p_temp.y=cloud1->points[i].y;
		p_temp.z=cloud1->points[i].z;
		p_temp.r=color_temp.r;
		p_temp.g=color_temp.g;
		p_temp.b=color_temp.b;
		P_error->push_back(p_temp);
		
		// confidence.txt
		fout<<p_temp.x<<" "<<p_temp.y<<" "<<p_temp.z<<" "<<cc.correspondance_[i].dist_<<endl;
	}
	
	pcl::io::savePLYFileASCII("P_error.ply",*P_error);
	fout.close();
}

float min_confidence=INT_MAX;
float max_confidence=-INT_MAX;
void load_confidence(string confidence_txt_filename)
{
	fin.open(confidence_txt_filename);
	if(fin.is_open()==false)
	{
		cout<<"Read file error!"<<endl;
	}
	while(getline(fin,line))
	{
		vector<string> ss=split(line," ");
		Point p_temp;
		p_temp.x_=atof(ss[0].c_str());
		p_temp.y_=atof(ss[1].c_str());
		p_temp.z_=atof(ss[2].c_str());
		p_temp.confidence_=atof(ss[3].c_str());
		min_confidence=(min_confidence<p_temp.confidence_) ? min_confidence: p_temp.confidence_;
		max_confidence=(max_confidence>p_temp.confidence_) ? max_confidence: p_temp.confidence_;
		ps_confidence.push_back(p_temp);
	}
	
	fin.close();
}

void extract_confidence(string P_opt_mapped_filename, string confidence_txt_filename)
{
	// load P_opt_mapped.ply 
	if(P_opt_mapped->points.size()==0)
		pcl::io::loadPLYFile<pcl::PointXYZRGBA>(P_opt_mapped_filename,*P_opt_mapped);
	
	// load confident.txt
	load_confidence(confidence_txt_filename);
	cout<<min_confidence<<" "<<max_confidence<<endl;
	
	
	CloudCompare cc;
	cc.find_correspondance(*P_opt_mapped,ps_confidence);
	fout.open("reproject_removal.txt");
	for(int i=0;i<P_opt_mapped->points.size();i++)
	{
		pcl::PointXYZRGBA p_temp;
		p_temp.x=P_opt_mapped->points[i].x;
		p_temp.y=P_opt_mapped->points[i].y;
		p_temp.z=P_opt_mapped->points[i].z;
		
		int cid=cc.correspondance_[i].d2_;
		V3 color_temp=get_color(min_confidence,max_confidence,ps_confidence[cid].confidence_);
		p_temp.r=color_temp.r;
		p_temp.g=color_temp.g;
		p_temp.b=color_temp.b;
		P_reproject_removal->push_back(p_temp);
		fout<<p_temp.x<<" "<<p_temp.y<<" "<<p_temp.z<<" "<<ps_confidence[cid].confidence_<<endl;
	}
	
	pcl::io::savePLYFileASCII("P_reproject_removal.ply",*P_reproject_removal);
	fout.close();
}

void extract_confidence_error(string confidence_filename, string error_filename)
{
	vector<Point> ps1,ps2;
	fin.open(confidence_filename);
	while(getline(fin,line))
	{
		vector<string> ss=split(line," ");
		Point p_temp;
		p_temp.x_=atof(ss[0].c_str());
		p_temp.y_=atof(ss[1].c_str());
		p_temp.z_=atof(ss[2].c_str());
		p_temp.confidence_=atof(ss[3].c_str());
		ps1.push_back(p_temp);
	}
	fin.close();
	
	
	fin.open(error_filename);
	while(getline(fin,line))
	{
		vector<string> ss=split(line," ");
		Point p_temp;
		p_temp.x_=atof(ss[0].c_str());
		p_temp.y_=atof(ss[1].c_str());
		p_temp.z_=atof(ss[2].c_str());
		p_temp.error_=atof(ss[3].c_str());
		ps2.push_back(p_temp);
	}
	fin.close();
	
	CloudCompare cc;
	cc.find_correspondance(ps1,ps2);
	fout.open("confidence_error.txt");
	for(int i=0;i<cc.correspondance_.size();i++)
	{
		int cid=cc.correspondance_[i].d2_;
		fout<<ps1[i].x_<<" "<<ps1[i].y_<<" "<<ps1[i].z_<<" "<<ps1[i].confidence_<<" "<<ps2[cid].error_<<endl;
	}
	fout.close();
}