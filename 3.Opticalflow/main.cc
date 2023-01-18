#include<iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include  "OpticalFlowTracker.h"
#include<chrono>

using namespace cv;
using std::vector;



/**
 * single level optical flow
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse = false,
    bool has_initial_guess = false
);


/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse = false
);



int main(int argc ,char** argv)
{
    // 读取灰度图像， 不是彩色图
    Mat img1 = imread("../LK1.png",0);
    Mat img2 = imread("../LK2.png",0);

    //提取关键点
    std::vector<KeyPoint> kp1;
    Ptr<GFTTDetector>detector  = GFTTDetector::create(500,0.01,20);  
    detector->detect(img1,kp1);

    vector<KeyPoint> kp2_single,kp2_multi;
    vector<bool> success_single,success_multi;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    OpticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout<<"Optical FLow SingleLevel cost   "<<time_used.count()<<" s   \n";

    t1 = std::chrono::steady_clock::now();
    OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi, success_multi, true);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout<<"Optical FLow MuitiLevel cost   "<<time_used.count()<<" s   \n";


    //------------------------opencv光流-------------------

    vector<Point2f> pt1,pt2;
    for(auto &kp :kp1)
        pt1.push_back(kp.pt);
    vector<uchar>status;
    vector<float>error;
    t1 = std::chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(img1,img2,pt1,pt2,status,error,Size(8,8),4);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout<<"opencv  cost     "<<time_used.count()<<" s   \n";




    




//------------------------------------------ test-------------------------------.
Mat img2_single;
cvtColor(img2,img2_single,CV_GRAY2BGR);
int successCount = 0;
for(int i=0 ; i<kp2_single.size();i++)
{
    if(success_single[i])
    {
        successCount++;
        circle(img2_single,kp2_single[i].pt,2,Scalar(0,255,0),2);
        cv::line(img2_single,kp1[i].pt,kp2_single[i].pt,Scalar(0,200,0));
    }
}
std::cout<<" one level success : "<<successCount<<std::endl;
successCount =0;

Mat img2_multi;
cvtColor(img2,img2_multi,CV_GRAY2BGR);
for(int i=0 ; i<kp2_single.size();i++)
{
    if(success_multi[i])
    {
        successCount++;
        circle(img2_multi,kp2_multi[i].pt,2,Scalar(0,255,0),2);
        cv::line(img2_multi,kp1[i].pt,kp2_multi[i].pt,Scalar(0,200,0));
    }
}

std::cout<<" multi level success : "<<successCount<<std::endl;

successCount =0;
Mat img2_cv;
cvtColor(img2,img2_cv,CV_GRAY2BGR);
for(int i=0;i <pt2.size();i++)
{
    if(status[i])
    {
            successCount++;
            cv::circle(img2_cv ,pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_cv, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
    }
}
std::cout<<"opencv  success"<<successCount<<std::endl;


imshow("one level ",img2_single);
imshow("multi level ",img2_multi);
imshow("cv   trakced",img2_cv);
 waitKey(0);

}



void OpticalFlowSingleLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse ,
    bool has_initial
)
{
    kp2.resize(kp1.size());
    success.resize(kp1.size());
    OpticalFlowTracker tracker(img1, img2, kp1, kp2, success, inverse, has_initial);

    parallel_for_(Range(0, kp1.size()),std::bind(&OpticalFlowTracker::calculateOpticalFLow, &tracker,  std::placeholders::_1));

    
                  

}

//多层光流实现
void OpticalFlowMultiLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse)
{
    //金字塔参数
    int pyramids = 4;
    double pyarid_scale = 0.5;
    double scales[4]{1.0,0.5,0.25,0.125};

    //提取图像金字塔
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    vector<Mat> pyr1,pyr2;   //image pyramids

    for(int i=0; i<pyramids;i++)
    {
        if(i ==0)
        {
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        }
        else
        {
            Mat img1_pyr , img_pyr2;
            resize(pyr1[i-1],img1_pyr,Size(pyr1[i-1].cols*pyarid_scale,pyr1[i-1].rows*pyarid_scale));
            resize(pyr2[i -1], img_pyr2,Size(pyr2[i - 1].cols * pyarid_scale, pyr2[i - 1].rows * pyarid_scale));
            // imshow("test",img1_pyr);
            // waitKey(1000);

            pyr1.emplace_back(std::move(img1_pyr));
            pyr2.emplace_back(std::move(img_pyr2));
                       
        }
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);

    std::cout<<"build pyramid time:   "<<time_used.count()<<std::endl;


    //coarse-to-fine LK tracking in pyramids
    vector<KeyPoint> kp1_pyr, kp2_pyr;
    //std::cout<<kp1[0].pt<<std::endl;
    //先从金字塔顶层，最低分辨率开始
    for(auto kp :kp1)  //修改数据 不拿引用就完了
    {
        kp.pt *= scales[pyramids-1];
        kp1_pyr.push_back(kp);
        kp2_pyr.push_back(kp);
    }
    //std::cout<<kp1_pyr[0].pt<<std::endl;


    for(int level = pyramids-1 ; level>=0 ; level--)
    {
        success.clear();
        t1 = std::chrono::steady_clock::now();
        OpticalFlowSingleLevel(pyr1[level],pyr2[level],kp1_pyr,kp2_pyr,success,inverse,true);  //有初解
        t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
        std::cout<<"track pyr   "<<level<<"   cost time :  "<<time_used.count()<<std::endl;

        //作为下一层的金字塔的特征点， 要乘回尺度
        if(level>0)
        {
            for(auto &kp:kp1_pyr)
            {
                kp.pt /= pyarid_scale;
            }
            for(auto &kp: kp2_pyr)
            {
                kp.pt /= pyarid_scale;
            }
        }
    }


    for(auto &kp : kp2_pyr)
        kp2.push_back(kp);



}
