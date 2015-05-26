#include "myData.h"
#include <fstream>
#include <string>
#include <cmath>
#include "SharedValue.h"

myData::myData(int n)
{
	p_width=640;        //设定图片高宽
	p_height=480;
	r = cvCreateMat( 3, 3, CV_64FC1);
	t = cvCreateMat( 3, 1, CV_64FC1);
	r_inv = cvCreateMat(3,3,CV_64FC1);
	z_near=z_far=0;
	cam_num = n;
}



void myData::Init_RTs()
{
	cvInitMatHeader(r,3,3,CV_64FC1,cam_r[cam_num]);
	cvInitMatHeader(r_inv,3,3,CV_64FC1,cam_r_inv[cam_num]);
	cvInitMatHeader(t,3,1,CV_64FC1,cam_center[cam_num]);


	double dist[10];
	for (int i=0;i<10;i++)  //求离R_CAM最近的5个相机，其中为方便程序编写，包括了他自己
	{
		dist[i]=pow(cam_center[cam_num][0]-cam_center[i][0],2)+pow(cam_center[cam_num][1]-cam_center[i][1],2)+pow(cam_center[cam_num][2]-cam_center[i][2],2);
	}


	int nearest_cam[5];
	for (int i=0;i<5;i++)
	{
		double min_dist;
		min_dist=999;
		int min_dist_id=0;
		for (int j=1;j<10;j++)
		{
			if(min_dist>dist[j]&&dist[j]!=-1)
			{
				min_dist=dist[j];
				min_dist_id=j;
			}
		}

		nearest_cam[i]=min_dist_id;
		dist[min_dist_id]=-1;
	}


	for (int i=0;i<4;i++)  // 初始化target_cam
	{
		tar_cam_num[i]=nearest_cam[i+1];
	}

	for (int i=0;i<4;i++)
	{
		for (int j=0;j<12;j++)
		{
			tar_RTs[i][j]=RTs[tar_cam_num[i]][j];
		}
	}

	for (int i=0;i<12;i++)
	{
		ref_RT[i]=RTs[cam_num][i];
	}


}


void myData::Init_Depth()  //初始化扫描深度区间
{
	for (int i=0;i<8;i++)
	{
		double z;
		double point[3];
		for (int j=0;j<3;j++)
		{
			point[j]=corner_vertices[i][j];
		}

		CvMat *three_dp_real = cvCreateMat(3,1,CV_64FC1);
		CvMat *three_dp_ref = cvCreateMat(3,1,CV_64FC1);

		cvInitMatHeader(three_dp_real,3,1,CV_64FC1,point);

		cvSub(three_dp_real,t,three_dp_ref);
		cvMatMulAdd(r,three_dp_ref,0,three_dp_ref);

		z=CV_MAT_ELEM(*three_dp_ref,double,2,0);

		if (z_far==0)
		{
			z_far=CV_MAT_ELEM(*three_dp_ref,double,2,0);
		}
		if (z_near==0)
		{
			z_near=CV_MAT_ELEM(*three_dp_ref,double,2,0);
		}

		if (z>z_near)
		{
			z_near=z;
		}
		if (z<z_far)
		{
			z=z_far;
		}
	}
}

void myData::InitVC_Points()
{
	VC_Points.resize(p_height);
	for (int i=0;i<p_height;i++)
	{
		VC_Points[i].resize(p_width);
	}
	for (int i=0 ; i<p_height ; i++)
	{
		for (int j=0 ; j<p_width ; j++)
		{
			VC_Point p;
			p.B = 0;
			p.G = 0;
			p.R = 0;
			p.Score = -1;
			p.z = 0;
			for (int i=0;i<11;i++)
			{
				p.cam[i]=0;
			}
			VC_Points[i][j]=p;
		}
	}
}



void myData::compute_true_RTs()
{
	CvMat *change = cvCreateMat(4,4,CV_64FC1);
	double change_value[]=
	{CV_MAT_ELEM(*r_inv,double,0,0),CV_MAT_ELEM(*r_inv,double,0,1),CV_MAT_ELEM(*r_inv,double,0,2),CV_MAT_ELEM(*t,double,0,0),
	CV_MAT_ELEM(*r_inv,double,1,0),CV_MAT_ELEM(*r_inv,double,1,1),CV_MAT_ELEM(*r_inv,double,1,2),CV_MAT_ELEM(*t,double,1,0),
	CV_MAT_ELEM(*r_inv,double,2,0),CV_MAT_ELEM(*r_inv,double,2,1),CV_MAT_ELEM(*r_inv,double,2,2),CV_MAT_ELEM(*t,double,2,0),
	0        ,               0              ,                0             ,                  1         
	};
	cvInitMatHeader(change,4,4,CV_64FC1,change_value);



	for (int i=0 ; i<4 ; i++)
	{

		CvMat *rt = cvCreateMat( 3, 4, CV_64FC1);
		cvInitMatHeader(rt,3,4,CV_64FC1,tar_RTs[i]);

		cvMatMulAdd( rt, change, 0, rt);

	}


	for (int i=0;i<12;i++)  //设置ref相机射影矩阵
	{
		ref_RT[i]=0;
	}
	ref_RT[0]=480;
	ref_RT[2]=-319.5;
	ref_RT[5]=-480;
	ref_RT[6]=-239.5;
	ref_RT[10]=-1;
}

void myData::compute_depthmap()
{
	Mat img1_tar;
	Mat img2_tar;
	Mat img3_tar;
	Mat img4_tar;
	Mat img_ref;


	string file_path[4];
	stringstream ss;
	string tcn_s;

	for (int i=0;i<4;i++)
	{
		int tcn;
		tcn=tar_cam_num[i]+1;
		ss<<tcn;
		ss>>tcn_s;
		ss.clear();
		ss.str("");
		if (tcn<10)
		{
			file_path[i]="F:/三维重建/kermit/data/0000"+tcn_s+".jpg";;
		}
		else if (tcn==10)
		{
			file_path[i]="F:/三维重建/kermit/data/000"+tcn_s+".jpg";;
		}
	}
	img1_tar=imread(file_path[0]);
	img2_tar=imread(file_path[1]);
	img3_tar=imread(file_path[2]);
	img4_tar=imread(file_path[3]);

	if (cam_num<9)
	{
		int n;
		string n_s;
		n=cam_num+1;
		ss<<n;
		ss>>n_s;
		ss.clear();
		ss.str("");
		img_ref=imread("F:/三维重建/kermit/data/0000"+n_s+".jpg");
	}
	else if (cam_num==9)
	{
		int n;
		string n_s;
		n=cam_num+1;
		ss<<n;
		ss>>n_s;
		ss.clear();
		ss.str("");
		img_ref=imread("F:/三维重建/kermit/data/000"+n_s+".jpg");
	}

	Mat rec_ref(5,5,CV_8U);
	Mat rec_tar1(5,5,CV_8U);
	Mat rec_tar2(5,5,CV_8U);
	Mat rec_tar3(5,5,CV_8U);
	Mat rec_tar4(5,5,CV_8U);

	CvMat *comp2to3 = cvCreateMat( 3, 3, CV_64FC1); //根据二维点位置以及深度计算三维点位置
	CvMat *comp2to3_inv = cvCreateMat( 3, 3, CV_64FC1); 
	CvMat *rt = cvCreateMat( 3, 4, CV_64FC1);
	CvMat *three_d_point = cvCreateMat( 4, 1, CV_64FC1);
	CvMat *three_d_point_temp = cvCreateMat( 3, 1, CV_64FC1); //用于存储计算中间值
	CvMat *two_d_point = cvCreateMat(3,1,CV_64FC1);



	//从近平面扫描到远平面
	for (double z=z_near;z>z_far;z=z-0.01)
	{
		for (int x=0;x<p_width;x++)
		{
			for (int y=0;y<p_height;y++)
			{
				double score[4];          //NCC匹配分数
				for (int i=0 ; i<4 ; i++)
				{
					score[i]=0;
				}

				int r_color,g_color,b_color;
				r_color=g_color=b_color=0;

				r_color = img_ref.at<Vec3b>(y,x)[2];
				g_color = img_ref.at<Vec3b>(y,x)[1];
				b_color = img_ref.at<Vec3b>(y,x)[0];


				Point2f p_ref;
				p_ref.x = x;
				p_ref.y = y;
				getRectSubPix( img_ref, Size(5,5), p_ref,rec_ref);  // ref相机对应的图像块



				double comp2to3_value[9]={480,0,-319.5*z,
					0,-480,-239.5*z,
					0,0,-1*z};
				cvInitMatHeader(comp2to3,3,3,CV_64FC1,comp2to3_value);
				cvInvert(comp2to3, comp2to3_inv, CV_SVD);
				double value_2d[] = {x, y,1};
				cvInitMatHeader(two_d_point,3,1,CV_64FC1,value_2d);
				cvMatMulAdd( comp2to3_inv, two_d_point, 0, three_d_point_temp);
				double x_3d,y_3d,scale;
				x_3d=CV_MAT_ELEM(*three_d_point_temp,double,0,0);
				y_3d=CV_MAT_ELEM(*three_d_point_temp,double,1,0);
				scale=CV_MAT_ELEM(*three_d_point_temp,double,2,0);

				x_3d=x_3d/scale;
				y_3d=y_3d/scale;

				

				for (int i=0 ; i<4; i++)
				{
					cvInitMatHeader(rt,3,4,CV_64FC1,tar_RTs[i-1]);

					double b[] = {x_3d, y_3d, z, 1};
					cvInitMatHeader( three_d_point, 4, 1, CV_64FC1, b);

					cvMatMulAdd( rt, three_d_point, 0, two_d_point);
					double x_2d,y_2d,z_2d;
					z_2d = CV_MAT_ELEM(* two_d_point,double,2,0);
					x_2d = CV_MAT_ELEM(* two_d_point,double,0,0);
					y_2d = CV_MAT_ELEM(* two_d_point,double,1,0);
					//PrintMat(two_d_point);


					x_2d = x_2d/z_2d;
					y_2d = y_2d/z_2d;
					z_2d = 1;

					Point2f p_2d;
					p_2d.x = x_2d;
					p_2d.y = y_2d;

					if (x_2d<p_width && x_2d>0 && y_2d<p_height && y_2d>0)
					{
						switch(i)
						{
						case 0:
							getRectSubPix( img1_tar, Size(5,5), p_2d,rec_tar1);
							score[0]=-1;
							break;
						case 1:
							getRectSubPix( img2_tar, Size(5,5), p_2d,rec_tar2 );
							score[1]=-1;
							break;
						case 2:
							getRectSubPix( img3_tar, Size(5,5), p_2d,rec_tar3 );
							score[2]=-1;
							break;
						case 3:
							getRectSubPix( img4_tar, Size(5,5), p_2d,rec_tar4 );
							score[3]=-1;
							break;
						default:
							break;
						}

					}


				}

				for(int i=0 ; i<4 ; i++)
				{
					Mat res(1,1,CV_32F);
					switch(i)
					{
					case 0:
						if (score[0]==-1)
						{
							matchTemplate( rec_ref,rec_tar1, res, CV_TM_CCOEFF_NORMED);
							score[0] = ((float *)(res.data))[0];
						}
						break;
					case 1:
						if (score[1]==-1)
						{
							matchTemplate( rec_ref,rec_tar2, res, CV_TM_CCOEFF_NORMED);
							score[1] = ((float *)(res.data))[0];
						}
						break;
					case 2:
						if (score[2]==-1)
						{
							matchTemplate( rec_ref,rec_tar3, res, CV_TM_CCOEFF_NORMED);
							score[2] = ((float *)(res.data))[0];
						}
						break;
					case 3:
						if (score[3]==-1)
						{
							matchTemplate( rec_ref,rec_tar4, res, CV_TM_CCOEFF_NORMED);
							score[3] = ((float *)(res.data))[0];
						}
						break;
					default:
						break;
					}
				}

				double max_score=0;
				for (int i=0 ; i<4 ; i++)
				{
					if (score[i] > max_score)
					{
						max_score=score[i];
					}
				}

				if (VC_Points[y][x].Score< max_score && max_score>=0.8 && max_score!=1 )
				{
					VC_Points[y][x].R = r_color;
					VC_Points[y][x].G = g_color;
					VC_Points[y][x].B = b_color;
					VC_Points[y][x].x = x_3d;
					VC_Points[y][x].y = y_3d;
					VC_Points[y][x].Score = max_score;
					VC_Points[y][x].z = z;
				}
			}

		}
		cout<<z<<endl;
	}

}

void myData::generate_depthmap()
{
	IplImage* img =  cvLoadImage("1.jpg",0);
	CvScalar p;
	//cvFlip(img);
	for(int i=0;i<img->width;i++)
	{
	   for(int j=0;j<img->height;j++)
		{
			p.val[0]=VC_Points[j][i].z*(-40);
			cvSet2D(img,j,i,p);

		}
	} 

	cvSaveImage("1.jpg",img,0);
	
}

void myData::generate_pointCloud()
{
	fstream f;
	f.open("2.txt",ios::out);
	for (int i = 0 ; i<VC_Points.size() ; i++)
	{
		for (int j = 0 ; j<VC_Points[i].size(); j++)
			if (VC_Points[i][j].Score != -1)
			{
				f<<VC_Points[i][j].x<<" "<<VC_Points[i][j].y<<" "<<VC_Points[i][j].z<<" "<<VC_Points[i][j].R<<" "<<VC_Points[i][j].G<<" "<<VC_Points[i][j].B<<endl;
			}
	}
	f.close();
}

