#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>


#include <chrono>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <opencv2/core/eigen.hpp>
//放在eigen后面，不然会报错
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

void ceres_BA(const vector< Point3f >& pts1,const vector< Point3f >& pts2);
void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

// 像素坐标转相机归一化坐标
Point2d pixel2cam ( const Point2d& p, const Mat& K );

void pose_estimation_3d3d (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R, Mat& t
);




struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST ( Point3f p1, Point3f p2 ) : _p1 ( p1 ), _p2 ( p2 ) {}
    // 残差的计算
    template <typename T>
    bool operator() (const T* const cere_r,const T* const cere_t,T* residual ) const
    {
        T p_1[3];
        T p_2[3];
        p_2[0]=T(_p2.x);
        p_2[1]=T(_p2.y);
        p_2[2]=T(_p2.z);
        ceres::AngleAxisRotatePoint(cere_r,p_2,p_1);
        p_1[0]=p_1[0]+cere_t[0];
        p_1[1]=p_1[1]+cere_t[1];
        p_1[2]=p_1[2]+cere_t[2];
    residual[0] = T(_p1.x)-p_1[0];
    residual[1] = T(_p1.y)-p_1[1];
    residual[2] = T(_p1.z)-p_1[2];
        return true;
    }
    const Point3f _p1, _p2;    //
};




int main ( int argc, char** argv )
{
    if ( argc != 5 )
    {
        cout<<"usage: pose_estimation_3d3d img1 img2 depth1 depth2"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    // 建立3D点
    Mat depth1 = imread ( argv[3], CV_LOAD_IMAGE_UNCHANGED );       // 深度图为16位无符号数，单通道图像
    Mat depth2 = imread ( argv[4], CV_LOAD_IMAGE_UNCHANGED );       // 深度图为16位无符号数，单通道图像
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<Point3f> pts1, pts2;

    for ( DMatch m:matches )
    {
        ushort d1 = depth1.ptr<unsigned short> ( int ( keypoints_1[m.queryIdx].pt.y ) ) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        ushort d2 = depth2.ptr<unsigned short> ( int ( keypoints_2[m.trainIdx].pt.y ) ) [ int ( keypoints_2[m.trainIdx].pt.x ) ];
        if ( d1==0 || d2==0 )   // bad depth
            continue;
        Point2d p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, K );
        Point2d p2 = pixel2cam ( keypoints_2[m.trainIdx].pt, K );
        float dd1 = float ( d1 ) /5000.0;
        float dd2 = float ( d2 ) /5000.0;
        pts1.push_back ( Point3f ( p1.x*dd1, p1.y*dd1, dd1 ) );
        pts2.push_back ( Point3f ( p2.x*dd2, p2.y*dd2, dd2 ) );
    }

    cout<<"3d-3d pairs: "<<pts1.size() <<endl;
    Mat R, t;
    pose_estimation_3d3d ( pts1, pts2, R, t );
    cout<<"ICP via SVD results: "<<endl;
    cout<<"R = "<<R<<endl;
    cout<<"t = "<<t<<endl;
    cout<<"R_inv = "<<R.t() <<endl;
    cout<<"t_inv = "<<-R.t() *t<<endl;
    //不是-t，注意注意

    cout<<"ceres_start"<<endl<<endl;
    ceres_BA(pts1,pts2);
    cout<<"g2o_ba_with_img2_point"<<endl;

    // verify p1 = R*p2 + t
    for ( int i=0; i<5; i++ )
    {
        cout<<"p1 = "<<pts1[i]<<endl;
        cout<<"p2 = "<<pts2[i]<<endl;
        cout<<"(R*p2+t) = "<<
            R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t
            <<endl;
        cout<<endl;
    }
}

void ceres_BA(const vector< Point3f >& pts1,const vector< Point3f >& pts2)
{   
    double cere_r[3];
    double cere_t[3];
    
    ceres::Problem problem;
    for ( int i=0; i<pts1.size(); i++ )
    {
        ceres::CostFunction* costfunction=new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,3,3,3>(new CURVE_FITTING_COST(pts1[i],pts2[i]));
        problem.AddResidualBlock(costfunction,nullptr,cere_r,cere_t);
	//nullptr是核函数
    }

    // 配置求解器
    ceres::Solver::Options options;     //
    options.linear_solver_type = ceres::DENSE_SCHUR;  //
    options.minimizer_progress_to_stdout = true;   // 

    ceres::Solver::Summary summary;                // 
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve ( options, &problem, &summary );  // 
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;
    cout<<summary.BriefReport() <<endl;
    cout<<"cere_t="<<cere_t[0]<<" "<<cere_t[1]<<" "<<cere_t[2]<<endl;
    //Eigen::AngleAxisd aa(cere_r[0],cere_r[1],cere_r[2]);
    cout<<"cere_r="<<endl<<cere_r[0]<<" "<<cere_r[1]<<" "<<cere_r[2]<<endl<<endl;

    
    double leng=sqrt(cere_r[0]*cere_r[0]+cere_r[1]*cere_r[1]+cere_r[2]*cere_r[2]);
    Eigen::Vector3d zhou(cere_r[0]/leng,cere_r[1]/leng,cere_r[2]/leng);
    Eigen::AngleAxisd aa(leng,zhou);
    cout<<aa.matrix()<<endl;
    cout<<endl;
    
}

void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
   // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

void pose_estimation_3d3d (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R, Mat& t
)
{
  //  --------------------------step1  去质心坐标
    // center of mass
  Point3f p1 ,p2;
  int N = pts1.size();
  for(int i=0; i<N;i++)
  {
    p1 += pts1[1];
    p2 += pts2[i];
  }
  p1 = Point3f(Vec3f(p1)/N);
  p2 = Point3f(Vec3f(p2)/N);

  vector<Point3f> q1(N),q2(N);
  for(int i=0; i<N;i++)
  {
    q1[i] = pts1[i] -p1;
    q2[i] = pts2[i] -p2;
  }

  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for(int i=0; i<N;i++)
  {
    W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
  }

  cout<<"W ="<<W<<endl;

  //SVD on W

 Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

   if (U.determinant() * V.determinant() < 0)
	{
        for (int x = 0; x < 3; ++x)
        {
            U(x, 2) *= -1;
        }
	}

  cout<<"U="<<U<<endl;
  cout<<"V="<<V<<endl;

 Eigen::Matrix3d R_ = U* ( V.transpose() );
  Eigen::Vector3d t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );

  cv::eigen2cv(R_,R);
cv::eigen2cv(t_,t);


    
}

