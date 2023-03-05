#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include<vector>

using namespace std;

int main() {
    //生成旋转矩阵
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotationVector(M_PI/4, Eigen::Vector3d(0,0,1));
    R = rotationVector.toRotationMatrix();
    cout<<R<<endl<<endl;

    //生成一系列原始点
    Eigen::Vector3d p1 = Eigen::Vector3d(1,2,3);
    Eigen::Vector3d p2 = Eigen::Vector3d(6,5,4);
    Eigen::Vector3d p3 = Eigen::Vector3d(8,7,9);
    vector<Eigen::Vector3d> pA = {p1, p2, p3};

    //生成旋转后的点
    vector<Eigen::Vector3d> pB;
    for(auto p:pA)
    {
        pB.emplace_back(R*p);
    }

    //求两个点云的中心
    Eigen::Vector3d qA = Eigen::Vector3d(0,0,0), qB = Eigen::Vector3d(0,0,0);
    for(int i = 0; i< pA.size(); i++)
    {
        for(int j = 0; j<3; j++)
        {
            qA[j] += pA[i][j];
            qB[j] += pB[i][j];
        }
    }
    qA = qA/pA.size();
    qB = qB/pB.size();

    //求去中心的点云
    for(int i = 0; i<pA.size(); i++)
    {
        pA[i] = pA[i]-qA;
        pB[i] = pB[i]-qB;
    }

    //求矩阵W
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(int i = 0; i<pA.size(); i++)
    {
        W += pA[i]*pB[i].transpose();
    }

    //SVD分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W,Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    

    Eigen::Matrix3d Rr = U*V.transpose();
    cout<<Rr<<endl;
}
