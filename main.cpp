#include <iostream>
#include "global.h"
#include "texture_mapping.h"
#include "points_extract.hpp"
#include "CloudCompare.hpp"
#include "extract_confidence_error.h"
#include "Confidence.h"
#include "txt_to_color_ply.h"


using namespace std;
int main(int argc,char** argv)
{
	if(argc>2 && string(argv[1])=="points_extract") // extract Popt from points3D.txt
	{
		if(argc==2 || string(argv[2])=="--help" || string(argv[2])=="?") // help message			
		{				
			cout<<"Description: This function extract optimal point cloud from points3D.txt."<<endl;			
			cout<<"Note: points3d.txt -> *.ply (x,y,z,r,g,b)!"<<endl;
			cout<<endl;
			cout<<"Usage:"<<endl;
			cout<<"\tpcfiler points_extract <txt filename> <ply filename>"<<endl;
			cout<<endl;
		}
		else
		{
			points_extract(string(argv[2]),string(argv[3]));
		}		
	}
	else if(argc>2 && string(argv[1])=="outlier_removal") // remove outlier with xyz format
	{
		if(argc==2 || string(argv[2])=="--help" || string(argv[2])=="?") // help message			
		{				
			/*
			cout<<"Description: This function extract optimal point cloud from points3D.txt."<<endl;			
			cout<<"Note: points3d.txt -> *.ply (x,y,z,r,g,b)!"<<endl;
			cout<<endl;
			cout<<"Usage:"<<endl;
			cout<<"\tpcfiler points_extract <ply filename> <xyz filename>"<<endl;
			cout<<endl;
			*/			
		}
		else
		{
			// points_extract(string(argv[2]),string(argv[3]));
		}
		cout<<"This function has not been realized."<<endl;
	}
	else if(argc>2 && string(argv[1])=="texture_mapping") // texture mapping from xyz to ply
	{
		if(argc==2 || string(argv[2])=="--help" || string(argv[2])=="?") // help message			
		{				
			cout<<"Description: This function mapped the texture from xyz to ply."<<endl;			
			cout<<"Note: the input ply and xyz file should be registered first!"<<endl;
			cout<<endl;
			cout<<"Usage:"<<endl;
			cout<<"\tpcfiler texture_mapping <ply filename> <xyz filename> <new ply filename>"<<endl;
			cout<<endl;
		}
		else
		{
			texture_mapping(string(argv[2]),string(argv[3]),string(argv[4]));
		}		
	}
	else if(argc>2 && string(argv[1])=="extract_error")
	{
		if(argc==2 || string(argv[2])=="--help" || string(argv[2])=="?") // help message			
		{				
			cout<<"Description: This function extract error."<<endl;			
			cout<<"Note: the input ply and xyz file should be registered first!"<<endl;			
			cout<<"Usage:"<<endl;
			cout<<"\tpcfiler extract_error <ground true ply filename> <xyz filename>"<<endl;
			cout<<endl;
		}
		else
		{
			 extract_error(string(argv[2]),string(argv[3]));	
		}
	}
	else if(argc>2 && string(argv[1])=="load_sparse") // extract confidence and error
	{
		if(argc==2 || string(argv[2])=="--help" || string(argv[2])=="?") // help message			
		{				
			cout<<"Description: Load sparse, export P_reproject.ply and confidence_with_outlier.txt."<<endl;			
			cout<<"Note: the input ply and xyz file should be registered first!"<<endl;			
			cout<<"Usage:"<<endl;
			cout<<"\tpcfiler extract_confidence <ply filename> <confidence_with_outlier.txt filename>"<<endl;
			cout<<"e.g."<<endl;
			cout<<"\t./pcfilter extract_confidence ../result/P_opt_mapped.ply ../result/confidence_with_outlier.txt"<<endl;
			cout<<endl;
		}
		else
		{			
			load_cameras_and_images(string(argv[2])); //export P_reproject.ply and confidence_with_outlier.txt			
		}		
	}
	else if(argc>2 && string(argv[1])=="extract_confidence") // extract confidence and error
	{
		if(argc==2 || string(argv[2])=="--help" || string(argv[2])=="?") // help message			
		{				
			cout<<"Description: This function extract confidence."<<endl;			
			cout<<"Note: the input ply and xyz file should be registered first!"<<endl;			
			cout<<"Usage:"<<endl;
			cout<<"\tpcfiler extract_confidence <ply filename> <confidence_with_outlier.txt filename>"<<endl;
			cout<<"e.g."<<endl;
			cout<<"\t./pcfilter extract_confidence ../result/P_opt_mapped.ply ../result/confidence_with_outlier.txt"<<endl;
			cout<<endl;
		}
		else
		{
			// ./pcfilter extract_confidence ../result/P_opt_mapped.ply ../result/confidence_with_outlier.txt
			//load_cameras_and_images(string(argv[2])); //export P_reproject.ply and confidence_with_outlier.txt
			 extract_confidence(string(argv[2]),string(argv[3]));			
		}		
	}
	else if(argc ==3 && string(argv[1])=="txt_to_color_ply")
	{
		if(argc==2 || string(argv[2])=="--help" || string(argv[2])=="?") // help message			
		{				
			cout<<"Description: This function extract ply file with color."<<endl;						
			cout<<endl;
			cout<<"Usage:"<<endl;
			cout<<"\tpcfiler txt_to_color_ply <txt filename>"<<endl;
			cout<<endl;
		}
		else
		{			 
			 txt_to_color_ply(string(argv[2]));		
		}		
	}
	else
	{
		cout<<"Choose the function you want to use!"<<endl;
		cout<<"*\tpoints_extract"<<endl;
		cout<<"*\ttexture_mapping"<<endl;
		cout<<"*\textract_confidence_error"<<endl;
	}
	return 0;
}