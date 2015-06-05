#include <iostream>
#include "myData.h"
#include "SharedValue.h"
using namespace std;

int main()
{
	myData depth_map2 = myData(2);

	readRTs();
	compute_cam_center();
	get_cv();
	depth_map2.Init_RTs();
	depth_map2.Init_Depth();
	depth_map2.InitVC_Points();
	depth_map2.compute_true_RTs();
	depth_map2.compute_depthmap();
	//depth_map2.generate_depthmap();
	depth_map2.generate_pointCloud();

}