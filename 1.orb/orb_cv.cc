#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<chrono>

using namespace cv;
using namespace std;

// ! aa
int main(int argc, char**argv)
{
    //  read image
    Mat img_1 =  imread("../1.png",CV_LOAD_IMAGE_COLOR);
    Mat img_2 =  imread("../2.png",CV_LOAD_IMAGE_COLOR);
    assert(img_1.data !=nullptr && img_2.data !=nullptr);

    // init
    vector<KeyPoint> keypoint_1 ,keypoint_2;
    Mat descripotrs_1 ,descriptors_2;
    Ptr<FeatureDetector>detector = ORB::create();  // factory function
    Ptr<DescriptorExtractor>descriptor = ORB::create();
    Ptr<DescriptorMatcher>mathcer = DescriptorMatcher::create("BruteForce-Hamming");  

    // -- first   detect  Oriented FAST corner
    
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    detector->detect(img_1,keypoint_1);
    detector->detect(img_2,keypoint_2);




    //  -- second  calculate FAST
    
    descriptor->compute(img_1,keypoint_1,descripotrs_1);
    descriptor->compute(img_2,keypoint_2,descriptors_2);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double>time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);

    cout<<"extractor ORB cost= "<<time_used.count()<<" seconds "<<endl;
    cout<<"orb numbers: "<<keypoint_1.size()<<endl;
    cout<<"orb numbers: "<<keypoint_2.size()<<endl;


    Mat outimg1;
    // drawKeypoints(img_1,keypoint_1,outimg1,Scalar::all(-1));
    // imshow("orb features",outimg1);
    // waitKey(0);

    // -- third BRIEF match  using Hamming distance
    vector<DMatch> matches;  
    t1 = chrono::steady_clock::now();
    mathcer->match(descripotrs_1,descriptors_2,matches);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<time_used.count()<<endl;

    // --- third match pair
    auto min_max = minmax_element(matches.begin(),matches.end(),
                                    [](const DMatch&m1, const DMatch&m2){return m1.distance<m2.distance;});
    
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    cout<<"min dist "<<min_dist<<endl;
    cout<<"max_dist "<<max_dist<<endl;

    // --- filtrarte
    vector<DMatch> good_matches;
    // cout<<matches.size()<<endl;
    // cout<<descripotrs_1.size<<endl;
    for(int i=0; i<descripotrs_1.rows;i++)
    {
        if(matches[i].distance<=max(2*min_dist,30.0))
            good_matches.emplace_back(matches[i]);
    }
    cout<<good_matches.size()<<endl;


    // -------------picture
    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img_1,keypoint_1,img_2,keypoint_2,matches,img_match);
    drawMatches(img_1,keypoint_1,img_2,keypoint_2,good_matches,img_goodmatch);

    imshow("all mathces",img_match);
    imshow("good_matches",img_goodmatch);
    waitKey(0);


    

    
}
