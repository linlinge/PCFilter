#include "Confidence.h"
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
vector<float> calculate_reprojection_error(vector<Point3f>& Pw,vector<Point2f>& Pp,Mat& intrinsic, Mat& rvec, Mat& tvec);


class Camera
{
	public:	
		int id_;		
		MatrixXd intrinsic_;
		MatrixXd extrinsic_;
		float k1_,k2_,p1_,p2_;
		
		Camera()
		{
			intrinsic_.resize(3,3);
			extrinsic_.resize(3,4);
		}
		Camera(float fx,float fy,float cx, float cy)
		{
			intrinsic_.resize(3,3);
			extrinsic_.resize(3,4);
			intrinsic_(0,0)=fx;
			intrinsic_(1,1)=fy;
			intrinsic_(0,2)=cx;
			intrinsic_(1,2)=cy;
			intrinsic_(2,2)=1;
			extrinsic_(2,3)=1;
		}
		void SetIntrinsic(float fx,float fy, float cx, float cy)
		{
			intrinsic_(0,0)=fx;
			intrinsic_(1,1)=fy;
			intrinsic_(0,2)=cx;
			intrinsic_(1,2)=cy;
			intrinsic_(2,2)=1;
		}
		void SetExtrinsic(float q0,float q1,float q2,float q3, float tx, float ty,float tz)
		{
			extrinsic_(0,0)=1-2*pow(q2,2)-2*pow(q3,2);
			extrinsic_(0,1)=2*(q1*q2-q0*q3);
			extrinsic_(0,2)=2*(q0*q2+q1*q3);
			extrinsic_(1,0)=2*(q0*q3+q1*q2);
			extrinsic_(1,1)=1-2*pow(q1,2)-2*pow(q3,2);
			extrinsic_(1,2)=2*(q2*q3-q0*q1);
			extrinsic_(2,0)=2*(q1*q3-q0*q2);
			extrinsic_(2,1)=2*(q0*q1+q2*q3);
			extrinsic_(2,2)=1-2*pow(q1,2)-2*pow(q2,2);
			extrinsic_(0,3)=tx;
			extrinsic_(1,3)=ty;
			extrinsic_(2,3)=tz;
		}
};

class Point3D
{
	public:
		int id_;
		V3 data_;
		V3 color_;
		vector<float> errors_;
		float confidence_;
		void SetData(float x,float y,float z,float r,float g,float b)
		{
			data_=V3(x,y,z);
			color_=V3(x,y,z);
		}
};
enum CameraModel{PINHOLE,OPENCV};


vector<Camera> cameras;
vector<Point3D> points;
vector<Point3D*> full_points_indices;
vector<Camera*> full_cameras_indices;
CameraModel type;



bool load_cameras_txt(string filename)
{
	// Definition
	ifstream fin;
	string line;
	
	// open file
	fin.open(filename);
	if(fin.is_open()==false)
	{
		cout<<"Read file error!"<<endl;
		return false;
	}
		
	// read file
	while(getline(fin,line))
	{
		// read data
		vector<string> elements=split(line," ");
		if(elements[0]!="#")
		{
			// Definition
			Camera camera_temp;			
			if(elements[1]=="PINHOLE")
			{
				// set camera intrinsic
				camera_temp.id_=atoi(elements[0].c_str());
				type=PINHOLE;
				camera_temp.SetIntrinsic(atof(elements[4].c_str()),atof(elements[5].c_str()),atof(elements[6].c_str()),atof(elements[7].c_str()));			
				cameras.push_back(camera_temp);
			}
			else if(elements[1]=="OPENCV")
			{
				camera_temp.id_=atoi(elements[0].c_str());
				type=OPENCV;
				camera_temp.SetIntrinsic(atof(elements[4].c_str()),atof(elements[5].c_str()),atof(elements[6].c_str()),atof(elements[7].c_str()));
				camera_temp.k1_=atof(elements[8].c_str());
				camera_temp.k2_=atof(elements[9].c_str());
				camera_temp.p1_=atof(elements[10].c_str());
				camera_temp.p2_=atof(elements[11].c_str());
				cameras.push_back(camera_temp);
			}			
		}
	}

	// sort cameras with id
	sort(cameras.begin(),cameras.end(),[](const Camera&e1,const Camera&e2){return e1.id_ < e2.id_;});
	
	// complete indices
	full_cameras_indices.resize(cameras[cameras.size()-1].id_+1);
	for(int i=0;i<cameras.size();i++)
	{
		full_cameras_indices[cameras[i].id_]=&cameras[i];
	}
	
	//close file
	fin.close();
	return true;
}


bool load_points3d_txt(string filename)
{
	// definition
	ifstream fin;
	string line;
	
	// open file
	fin.open(filename);
	if(fin.is_open()==false)
	{
		cout<<"Read file error!"<<endl;
		return false;
	}
	
	// read file
	while(getline(fin,line))
	{
		// read data
		vector<string> elements=split(line," ");
		if(elements[0]!="#")
		{
			Point3D p_temp;
			p_temp.id_=atoi(elements[0].c_str());
			p_temp.data_.x=atof(elements[1].c_str());
			p_temp.data_.y=atof(elements[2].c_str());
			p_temp.data_.z=atof(elements[3].c_str());
			p_temp.color_.r=atoi(elements[4].c_str());
			p_temp.color_.g=atoi(elements[5].c_str());
			p_temp.color_.b=atoi(elements[6].c_str());
			points.push_back(p_temp);
		} 
	}			
	
	// close file
	fin.close();
	// sort points
	sort(points.begin(),points.end(),[](const Point3D& e1, Point3D& e2){ return e1.id_<e2.id_;});
	full_points_indices.resize(points[points.size()-1].id_+1);
	for(int i=0;i<points.size();i++)
	{
		full_points_indices[points[i].id_]=&points[i];
	}
	
	return true;
}


bool load_images_txt(string filename)
{
	// Definition
	ifstream fin;
	string line;
	MatrixXd x(2,1);		// pixel coordinate	
    int correspond_point_id=-INT_MAX;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m1_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m2_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);	
	pcl::PointXYZRGBA point_temp;
	
	// open file
	fin.open(filename);
	if(fin.is_open()==false)
	{
		cout<<"Read file error!"<<endl;
		return false;
	}
	
	// read file
	while(getline(fin,line))
	{
		// read data
		vector<string> elements=split(line," ");
		if(elements[0]!="#")
		{
			// 1st line
			int camera_id=atoi(elements[8].c_str());
			//cout<<"camera_id:"<<camera_id<<endl;
			float q0=atof(elements[1].c_str());
			float q1=atof(elements[2].c_str());
			float q2=atof(elements[3].c_str());
			float q3=atof(elements[4].c_str());
			float tx=atof(elements[5].c_str());
			float ty=atof(elements[6].c_str());
			float tz=atof(elements[7].c_str());
			full_cameras_indices[camera_id]->SetExtrinsic(q0,q1,q2,q3,tx,ty,tz);
						
			// 2nd line
			MatrixXd& tmp1=full_cameras_indices[camera_id]->intrinsic_;
			cv::Mat intrinsic=(cv::Mat_<double>(3,3) << tmp1(0,0) , tmp1(0,1) , tmp1(0,2) , tmp1(1,0) , tmp1(1,1) , tmp1(1,2) , tmp1(2,0) , tmp1(2,1) , tmp1(2,2));
			MatrixXd& tmp2=full_cameras_indices[camera_id]->extrinsic_;
			cv::Mat rtmp=(cv::Mat_<double>(3,3) << tmp2(0,0) , tmp2(0,1) , tmp2(0,2) , tmp2(1,0) , tmp2(1,1) , tmp2(1,2) , tmp2(2,0) , tmp2(2,1) , tmp2(2,2));
			cv::Mat rvec;
			Rodrigues(rtmp,rvec);
			cv::Mat tvec=(cv::Mat_<double>(3,1) << tmp2(0,3) , tmp2(1,3) , tmp2(2,3));
			
			
			// A=full_cameras_indices[camera_id]->intrinsic_*full_cameras_indices[camera_id]->extrinsic_; // calculate projection matrix
			getline(fin,line);
			elements.clear();
			elements=split(line," ");
			for(int i=0;3*i < elements.size();i++)
			{
				correspond_point_id=atoi(elements[3*i+2].c_str());
				if(correspond_point_id!=-1)
				{	
					Point3f ptmp;
					ptmp.x=full_points_indices[correspond_point_id]->data_.x;
                    ptmp.y=full_points_indices[correspond_point_id]->data_.y;
                    ptmp.z=full_points_indices[correspond_point_id]->data_.z;
					vector<cv::Point3f> Pw;
					Pw.push_back(ptmp);
					
					x(0,0)=atof(elements[3*i].c_str());	
					x(1,0)=atof(elements[3*i+1].c_str());
					vector<cv::Point2f> reProjection;
					
					Camera& ctmp = *full_cameras_indices[camera_id];
					cv::Mat distCoff=(cv::Mat_<double>(1,4) << ctmp.k1_, ctmp.k2_, ctmp.p1_, ctmp.p2_);
					projectPoints(Pw,rvec,tvec,intrinsic, distCoff,reProjection);													
					// calculate projection error
                    float cost=sqrt(pow(x(0,0)-reProjection[0].x,2)+pow(x(1,0)-reProjection[0].y,2));					
					full_points_indices[correspond_point_id]->errors_.push_back(cost);										
				}
			}
		}
	}
	cout<<"end!"<<endl;
	
	// m2
	//pcl::io::savePLYFileASCII("m2_cloud.ply",*m2_cloud);
	
	// m1
	// calculate for average error
	// write confidence to file
	float Cmin,Cmax;
	Cmin=INT_MAX;
	Cmax=-INT_MAX;
	for(int i=0;i<full_points_indices.size();i++)
	{
		if(full_points_indices[i]!=NULL)
		{
			float sum=0;
			for(int j=0;j<full_points_indices[i]->errors_.size();j++)
			{
				sum+=full_points_indices[i]->errors_[j];
			}
			full_points_indices[i]->confidence_=sum/full_points_indices[i]->errors_.size();
			Cmin=(Cmin < full_points_indices[i]->confidence_) ? Cmin:full_points_indices[i]->confidence_;
			Cmax=(Cmax > full_points_indices[i]->confidence_) ? Cmax:full_points_indices[i]->confidence_;
		}		
	}	
	cout<<"Cmin: "<<Cmin<<endl;
	cout<<"Cmax: "<<Cmax<<endl;
	// write confidence to file
	ofstream fout("confidence_with_outlier.txt");
	
	m1_cloud->clear();
	for(int i=0;i<full_points_indices.size();i++)
	{
		if(full_points_indices[i]!=NULL)
		{
			point_temp.x=full_points_indices[i]->data_.x;
			point_temp.y=full_points_indices[i]->data_.y;
			point_temp.z=full_points_indices[i]->data_.z;
			V3 color_temp=get_color(Cmin,Cmax,full_points_indices[i]->confidence_);			
			point_temp.r=color_temp.r;
			point_temp.g=color_temp.g;
			point_temp.b=color_temp.b;
			m1_cloud->push_back(point_temp);
			fout<<point_temp.x<<" "<<point_temp.y<<" "<<point_temp.z<<" "<<full_points_indices[i]->confidence_<<endl;
		}
	}
	pcl::io::savePLYFileASCII("P_reproject.ply",*m1_cloud);	// save ply file
		
	// close file
	fin.close();
	fout.close();
	return true;
}

bool load_cameras_and_images(string path)
{
	string camera_filename=path+"/cameras.txt";
	string point_filename=path+"/points3D.txt";
	string image_filename=path+"/images.txt";
	
	load_cameras_txt(camera_filename);
	load_points3d_txt(point_filename);
	load_images_txt(image_filename);
	return true;
}

vector<float> calculate_reprojection_error(vector<Point3f>& Pw,vector<Point2f>& Pp,Mat& intrinsic, Mat& rvec, Mat& tvec)
{
	vector<cv::Point2f> reProjection;
	vector<float> reProjectionError;
	cv::projectPoints(Pw,rvec,tvec,intrinsic,cv::Mat(),reProjection);
	//cout<<"size:"<<reProjection.size()<<endl;
	
	for(int i=0;i<reProjection.size();i++)
	{		
		float error=sqrt(pow(Pp[i].x-reProjection[i].x,2)+pow(Pp[i].y-reProjection[i].y,2));
		reProjectionError.push_back(error);	
	}
	
	return reProjectionError;
}
