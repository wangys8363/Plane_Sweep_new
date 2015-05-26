#include "SharedValue.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <opencv2\opencv.hpp>
#include <core\core.hpp>
using namespace cv;
using namespace std;

double RTs[10][12]; 

double cam_r[10][9]; 

double cam_r_inv[10][9];

double cam_center[10][3];

double corner_vertices[8][3];


void readRTs()
{
	fstream read;
	int i ;
	stringstream ss;
	string i_str;
	string path;
	string strLine;

	for (i=1 ; i<=10 ; i++)
	{
		ss<<i;
		ss>>i_str;
		ss.clear();
		ss.str("");
		if (i<10)
		{
			path = "F:/三维重建/kermit/data/0000"+ i_str + "_P.txt";

		}
		else if (i<100)
		{
			path = "F:/三维重建/kermit/data/000"+ i_str + "_P.txt";

		}
		read.open(path);
		getline(read,strLine);
		int n = 0;
		while(getline(read,strLine))
		{
			istringstream is(strLine);
			is >> RTs[i-1][n] >> RTs[i-1][n+1] >> RTs[i-1][n+2] >> RTs[i-1][n+3];
			n += 4;
		}

		read.close();

	}

}

void compute_cam_center()
{
	CvMat *r;
	CvMat *t;
	CvMat *r_inv;

	r = cvCreateMat( 3, 3, CV_64FC1);
	t = cvCreateMat( 3, 1, CV_64FC1);
	r_inv = cvCreateMat(3,3,CV_64FC1);

	for (int i=0;i<10;i++)
	{
		double RT[12];
		for (int j=0;j<12;j++)
		{
			RT[j]=RTs[i][j];
		}

		CvMat *rt_original = cvCreateMat( 3, 4, CV_64FC1);


		double b[] ={-480, 0 ,319.5,
			0 ,480,239.5,
			0 , 0 , 1};


		cvInitMatHeader(rt_original,3,4,CV_64FC1,RT);

		CvMat *k = cvCreateMat( 3, 3, CV_64FC1);
		CvMat *k_inv = cvCreateMat( 3, 3, CV_64FC1);

		cvInitMatHeader(k,3,3,CV_64FC1,b);


		cvInvert(k, k_inv, CV_LU);

		cvMatMulAdd( k_inv, rt_original, 0, rt_original);

		CvMat *fe = cvCreateMat( 3, 3, CV_64FC1);
		double fe_value[]={-1,0,0,
			0,-1,0,
			0,0,-1};
		cvInitMatHeader(fe,3,3,CV_64FC1,fe_value);


		cvMatMulAdd(fe,rt_original,0,rt_original);

		double x,y,z;
		x=CV_MAT_ELEM(*rt_original,double,0,3);
		y=CV_MAT_ELEM(*rt_original,double,1,3);
		z=CV_MAT_ELEM(*rt_original,double,2,3);

		double t_value[]={-x,
			-y,
			-z};

		cvInitMatHeader(t,3,1,CV_64FC1,t_value);


		double r_value[]={CV_MAT_ELEM(*rt_original,double,0,0),CV_MAT_ELEM(*rt_original,double,0,1),CV_MAT_ELEM(*rt_original,double,0,2),
			CV_MAT_ELEM(*rt_original,double,1,0),CV_MAT_ELEM(*rt_original,double,1,1),CV_MAT_ELEM(*rt_original,double,1,2),
			CV_MAT_ELEM(*rt_original,double,2,0),CV_MAT_ELEM(*rt_original,double,2,1),CV_MAT_ELEM(*rt_original,double,2,2)};
		cvInitMatHeader(r,3,3,CV_64FC1,r_value);

		cvInvert(r, r_inv, CV_LU);

		cvMatMulAdd(r_inv,t,0,t);


		for (int y=0;y<3;y++)
		{
			for (int x=0;x<3;x++)
			{ 
				cam_r[i][y*3+x]=CV_MAT_ELEM(*r,double,y,x);
			}
		}

		for (int y=0;y<3;y++)
		{
			for (int x=0;x<3;x++)
			{ 
				cam_r_inv[i][y*3+x]=CV_MAT_ELEM(*r_inv,double,y,x);
			}
		}

		for (int y=0;y<3;y++)
		{ 
			cam_center[i][y]=CV_MAT_ELEM(*t,double,y,0);
		}
	}
}


void get_cv()
{
	fstream read;
	read.open("F:/三维重建/kermit/bundle/zFinal.ply");
	string strLine;
	double max_x,min_x,max_y,min_y,max_z,min_z;
	for (int i=0;i<12;i++)
	{
		getline(read,strLine);
	}
	getline(read,strLine);
	istringstream is(strLine);
	is>>max_x>>max_y>>max_z;
	min_x=max_x;
	min_y=max_y;
	min_z=max_z;
	is.clear();
	is.str("");
	while (getline(read,strLine))
	{
		double x,y,z;
		is.str(strLine);
		is>>x>>y>>z;
		if (x<min_x)
		{
			min_x=x;
		}
		if (x>max_x)
		{
			max_x=x;
		}
		if (y<min_y)
		{
			min_y=y;
		}
		if (y>max_y)
		{
			max_y=y;
		}
		if (z<min_z)
		{
			min_z=z;
		}
		if (z>max_z)
		{
			max_z=z;
		}
	}

	corner_vertices[0][0]=min_x;  corner_vertices[0][1]=min_y;  corner_vertices[0][2]=min_z;

	corner_vertices[1][0]=min_x;  corner_vertices[1][1]=max_y;  corner_vertices[1][2]=min_z;

	corner_vertices[2][0]=max_x;  corner_vertices[2][1]=min_y;  corner_vertices[2][2]=min_z;

	corner_vertices[3][0]=max_x;  corner_vertices[3][1]=max_y;  corner_vertices[3][2]=min_z;


	corner_vertices[4][0]=min_x;  corner_vertices[4][1]=min_y;  corner_vertices[4][2]=max_z;

	corner_vertices[5][0]=min_x;  corner_vertices[5][1]=max_y;  corner_vertices[5][2]=max_z;

	corner_vertices[6][0]=max_x;  corner_vertices[6][1]=min_y;  corner_vertices[6][2]=max_z;

	corner_vertices[7][0]=max_x;  corner_vertices[7][1]=max_y;  corner_vertices[7][2]=max_z;
}