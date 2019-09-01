#include "txt_to_color_ply.h"
void txt_to_color_ply(string txt_filename)
{
	cout<<txt_filename<<endl;
	fin.open(txt_filename);
	if(fin.is_open()==false)
	{
		cout<<"Read file error!"<<endl;
	}
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGBA>);
	vector<Point> ps;
	float Cmin=INT_MAX;
	float Cmax=-INT_MAX;
	while(getline(fin,line))
	{
		vector<string> ss=split(line," ");
		Point ptmp;
		ptmp.x_=atof(ss[0].c_str());
		ptmp.y_=atof(ss[1].c_str());
		ptmp.z_=atof(ss[2].c_str());
		ptmp.confidence_=atof(ss[3].c_str());		
		Cmin=(Cmin< ptmp.confidence_) ? Cmin:ptmp.confidence_;
		Cmax=(Cmax> ptmp.confidence_) ? Cmax:ptmp.confidence_;
		ps.push_back(ptmp);
	}
	
	cout<<ps.size()<<endl;
	for(int i=0;i<ps.size();i++)
	{
		pcl::PointXYZRGBA ptmp;
		ptmp.x=ps[i].x_;
		ptmp.y=ps[i].y_;
		ptmp.z=ps[i].z_;
		V3 color_tmp=get_color(Cmin,Cmax,ps[i].confidence_);
		ptmp.r=color_tmp.r;
		ptmp.g=color_tmp.g;
		ptmp.b=color_tmp.b;
		pc->push_back(ptmp);
	}
	
	pcl::io::savePLYFileASCII("P_reproject_removal_color.ply",*pc);
	fin.close();
}