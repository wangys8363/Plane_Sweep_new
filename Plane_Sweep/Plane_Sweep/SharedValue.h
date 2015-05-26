extern double RTs[10][12];  //原始射影矩阵

extern double cam_r[10][9];  //相机旋转

extern double cam_r_inv[10][9]; //相机旋转的逆

extern double cam_center[10][3];  // 相机光心

extern double corner_vertices[8][3]; //由稀疏点云确定的包络的8个顶点

void readRTs();//读取原始射影矩阵

void compute_cam_center(); //计算相机光心


void get_cv(); // 初始化包络顶点

