#ifndef __OpticalFlowTracker__
#define  __OpticalFlowTracker__
#include<iostream>
#include "opencv2/opencv.hpp"
#include  "Eigen/Core"


using namespace cv;
using std::vector;

class OpticalFlowTracker
{
    public:
        OpticalFlowTracker(
                const Mat &img1_,
                const Mat &img2_,
                const vector<KeyPoint> &kp1_,
                vector<KeyPoint> &kp2_,
                vector<bool> &success_,
                bool inverse_ = true, bool has_initial_ = false) :
                img1(img1_), img2(img2_), kp1(kp1_), kp2(kp2_), success(success_), inverse(inverse_),
                has_initial(has_initial_) {}
        OpticalFlowTracker(const OpticalFlowTracker& other) = delete;
        OpticalFlowTracker  operator=(const OpticalFlowTracker & other) = delete;

                void calculateOpticalFLow(const Range &range);
                void test();




    private:
        const Mat &img1;
        const Mat &img2;
        const vector<KeyPoint> &kp1;
        vector<KeyPoint> &kp2;
        vector<bool> &success;
        bool inverse = true;
        bool has_initial = false;
};










#endif
