# 视觉SLAM第7讲

## pose_estimation_3d2d  PnP
*  输入N组匹配好的特征点和图像对应的3D点坐标， 输出两幅图像之间的位姿变换
* 误差为重投影误差，优化的是位姿，dx为 Vector6d类型

## pose_estimatetion_3d3d P3P

* 输入，两组对应图像的匹配特征点和两幅图像的3D点，输出为两幅图像之间的位姿变换
* 有了坐标，把像素坐标投影回相机坐标系，在相机坐标系里求解
* opencv 里有SVD分解函数，可以用去质心坐标调用SVD分解
* 使用ceres求解
    * 残差维度是3，分别是 x y z 三个方向的坐标差
    * 优化变量两个，分别是角度r 三维， 平移t 三维
    * 也可以使用李代数 Sophus库来同时优化R和t
    * 优化之后通过角轴再变换到旋转矩阵

## 三角化
* 调用opencv对应函数求解本质矩阵，基础矩阵，三角化
