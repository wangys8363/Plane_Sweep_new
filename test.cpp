#include <iostream>
#include "depthmap_file_io.h"

int main(int argc, char const *argv[])
{
	//ʹ��˵��
	//���ͼ�����ݿ�����float��double
	DepthmapFileIO<float> depthmap1;
	//���ý�ƽ���Զƽ��
	depthmap1.SetNearFar(0.0,10.0);
	//����ͼ��Ŀ�͸�
	depthmap1.SetWidthHeight(2,2);

	//������ݣ�����
	std::vector<float> data;
	data.push_back(1.0);
	data.push_back(2.0);
	data.push_back(3.0);
	data.push_back(4.0);
	depthmap1.SetData(data);

	//�����ͼ���浽�ļ�
	depthmap1.SaveTofile("d1.out");

	//���ļ���ȡ���ͼ��Ϣ
	DepthmapFileIO<float> depthmap2;
	depthmap2.LoadFromfile("d1.out");


	return 0;
}