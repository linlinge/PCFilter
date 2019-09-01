#include "texture_mapping.h"
void texture_mapping(string in_ply_filename, string xyz_filename, string out_ply_filename)
{
	// ply read
	// definition
	int vertex_number=0;
	int flag=0; // end_header exist or not
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr old_ply(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPLYFile<pcl::PointXYZRGBA>(in_ply_filename, *old_ply);
	
	// xyz read
	fin.open(xyz_filename);
	if(fin.is_open()==false)
	{
		cout<<"Load xyz file error!"<<endl;
	}
	while(getline(fin,line))
	{
		vector<string> ss=split(line," ");
		Point p_temp;
		p_temp.x_=atof(ss[0].c_str());
		p_temp.y_=atof(ss[1].c_str());
		p_temp.z_=atof(ss[2].c_str());
		ps_xyz.push_back(p_temp);
	}
	// close xyz file
	fin.close();
	
	// match
	vector<Point> ps_new_ply;
	//#pragma omp parallel for
	for(int i=0;i<ps_xyz.size();i++)
	{
		float min_dist=INT_MAX;
		Point p_temp;
			
		for(int j=0;j<old_ply->points.size();j++)
		{			
			float dist=Distance(ps_xyz[i],old_ply->points[j]);			
			if(min_dist>dist)
			{				
				min_dist=dist;				
				p_temp.r_=old_ply->points[j].r;
				p_temp.g_=old_ply->points[j].g;
				p_temp.b_=old_ply->points[j].b;
				//cout<<min_dist<<endl;
			}		
			if(dist < 0.00001)
			{
				break;
			}
			
		}
		p_temp.x_=ps_xyz[i].x_;
		p_temp.y_=ps_xyz[i].y_;
		p_temp.z_=ps_xyz[i].z_;		
		ps_new_ply.push_back(p_temp);
	}
	
	// write ply file
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ps(new pcl::PointCloud<pcl::PointXYZRGBA>);
	for(int i=0;i<ps_new_ply.size();i++)
	{
		pcl::PointXYZRGBA p_temp;
		p_temp.x=ps_new_ply[i].x_;
		p_temp.y=ps_new_ply[i].y_;
		p_temp.z=ps_new_ply[i].z_;
		p_temp.r=ps_new_ply[i].r_;
		p_temp.g=ps_new_ply[i].g_;
		p_temp.b=ps_new_ply[i].b_;
		P_opt_mapped->push_back(p_temp);
	}
	pcl::io::savePLYFileASCII(out_ply_filename,*P_opt_mapped);
}