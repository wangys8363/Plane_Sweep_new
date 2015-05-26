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
	int p_width,p_height;  //��Ƭ�߿�
	double z_near,z_far;  //ɨ�����
	int cam_num;  //��ǰref_cam�����
	int tar_cam_num[4];  // ��ǰtar_cam�����
	

public:

	double tar_RTs[4][12]; //target�������
	double ref_RT[12]; //reference�������
	CvMat *t;  // ��ǰ�ο�����Ĺ���
	CvMat *r;  // ��ǰ�ο��������תR
	CvMat *r_inv; //r��
	vector <vector <VC_Point>> VC_Points; //�����Ϣ

	


	myData(int n);

	void Init_RTs();

	void Init_Depth();

	void InitVC_Points();
	void compute_true_RTs();

	void compute_depthmap();

	void generate_depthmap();
	void generate_pointCloud();
};

