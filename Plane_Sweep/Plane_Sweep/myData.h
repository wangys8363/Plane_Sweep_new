#pragma  once
#include <vector>
#include <opencv2\opencv.hpp>
#include <core\core.hpp>
using namespace cv;
using namespace std;

struct VC_Point
{
	double Score;
	double z;
	double x;
	double y;
	int R;
	int G;
	int B;
	int cam[11];
};


class myData
{
private:
	int p_width,p_height;  //照片高宽
	double z_near,z_far;  //扫描深度
	int cam_num;  //当前ref_cam的序号
	int tar_cam_num[4];  // 当前tar_cam的序号
	

public:

	double tar_RTs[4][12]; //target相机矩阵
	double ref_RT[12]; //reference相机矩阵
	CvMat *t;  // 当前参考相机的光心
	CvMat *r;  // 当前参考相机的旋转R
	CvMat *r_inv; //r逆
	vector <vector <VC_Point>> VC_Points; //深度信息

	


	myData(int n);

	void Init_RTs();

	void Init_Depth();

	void InitVC_Points();
	void compute_true_RTs();

	void compute_depthmap();

	void generate_depthmap();
	void generate_pointCloud();
};

