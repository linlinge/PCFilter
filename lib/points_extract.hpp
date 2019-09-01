#include "global.h"

void points_extract(string points3d_filename, string out_ply_filename)
{
	// open file
	fin.open(points3d_filename);
	if(fin.is_open()==false)
	{
		cout<<"Read file error!"<<endl;
		return;
	}

	// read file
	while(getline(fin,line))
	{
		//definition
		vector<string> elements=split(line," ");
		if(elements[0]!="#")
		{
			pcl::PointXYZRGBA point_temp;
			point_temp.x=atof(elements[1].c_str());
			point_temp.y=atof(elements[2].c_str());
			point_temp.z=atof(elements[3].c_str());
			point_temp.r=atoi(elements[4].c_str());
			point_temp.g=atoi(elements[5].c_str());
			point_temp.b=atoi(elements[6].c_str());
			P_opt->push_back(point_temp);
		}			
	}
	pcl::io::savePLYFileASCII(out_ply_filename,*P_opt);
	cout<<"Numbers: "<<P_opt->points.size()<<endl;
	//close file
	fin.close();
}