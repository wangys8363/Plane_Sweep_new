#include <iostream>
#include "depthmap_file_io.h"

int main(int argc, char const *argv[])
{
	//使用说明
	//深度图，数据可以是float或double
	DepthmapFileIO<float> depthmap1;
	//设置近平面和远平面
	depthmap1.SetNearFar(0.0,10.0);
	//设置图像的宽和高
	depthmap1.SetWidthHeight(2,2);

	//深度数据，数组
	std::vector<float> data;
	data.push_back(1.0);
	data.push_back(2.0);
	data.push_back(3.0);
	data.push_back(4.0);
	depthmap1.SetData(data);

	//将深度图保存到文件
	depthmap1.SaveTofile("d1.out");

	//从文件读取深度图信息
	DepthmapFileIO<float> depthmap2;
	depthmap2.LoadFromfile("d1.out");


	return 0;
}