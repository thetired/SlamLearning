#include "OpticalFlowTracker.h"
#include "Eigen/Core"
#include"Eigen/Dense"

namespace
{
    inline double GetPixelValue (const cv::Mat&img ,float x, float y)
    {
        if(x<0) x=0;
        if(y<0)y=0;
        if(x>=img.cols-1) x= img.cols-2;
        if(y>=img.rows-1)y=img.rows-2;


        float xx = x-floor(x);
        float yy = y-floor(y);
        int x_a1  =std::min(img.cols-1 ,int(x)+1);
        int y_a1 = std::min(img.rows-1,int(y)+1);
          return (1 - xx) * (1 - yy) * img.at<uchar>(y, x)
                        + xx * (1 - yy) * img.at<uchar>(y, x_a1)
                        + (1 - xx) * yy * img.at<uchar>(y_a1, x)
                        + xx * yy * img.at<uchar>(y_a1, x_a1);
    }
}

 void OpticalFlowTracker::calculateOpticalFLow(const Range &range)
 {
    int halt_patch_size = 4;
    int iterations = 10;
 //遍历所有keypoint
    for(size_t i= range.start ; i<range.end ;i++)  
    {
        //dx dy 是待估计参数， 类似于 a 和b
        auto kp = kp1[i];
        double dx=0 ,dy =0;     
        // for  多层光流用的
        if(this->has_initial)
        {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y-kp.pt.y;
        }

        double cost =0 ,lastCost = 0;
        bool  succ = true; 

        // Gauss Newton
        Eigen::Matrix2d  H = Eigen::Matrix2d::Zero();
        Eigen::Vector2d J;
        Eigen::Vector2d b = Eigen::Vector2d::Zero();

        for(int iter =0 ; iter<iterations;iter++)
        {  //迭代次数
            if(this->inverse == false)
            {  //正常高斯牛顿，每次迭代H和b从置
                H = Eigen::Matrix2d::Zero();
                 b = Eigen::Vector2d::Zero();
                 J = Eigen::Vector2d::Zero();
            }
            else
                 b = Eigen::Vector2d::Zero();
                
            cost = 0 ;

            // 每一次迭代时遍历的点数， LK光iu法
            for(int x = - halt_patch_size ; x<halt_patch_size; x++)
            {  
                for(int y = -halt_patch_size ; y<halt_patch_size; y++)
                {
                    //得到f(x)
                    double error = GetPixelValue(img1,kp.pt.x+x,kp.pt.y+y) -
                                                    GetPixelValue(img2,kp.pt.x+x+dx,kp.pt.y+y+dy);
                    if(this->inverse == false)
                    { //对于反向光流，不更新雅可比

                        J = -1.0 *Eigen::Vector2d(
                                    0.5*(GetPixelValue(img2,kp.pt.x+dx+x+1,kp.pt.y+dy+y) -
                                             GetPixelValue(img2,kp.pt.x+dx+x-1,kp.pt.y+y+dy)),
                                    0.5*(GetPixelValue(img2,kp.pt.x+dx+x,kp.pt.y+dy+1)-
                                                GetPixelValue(img2,kp.pt.x+dx+x,kp.pt.y+dy-1)));
                    }
                    else if (iter ==0)       //逆向光流， 雅可比矩阵不更新，只更新误差
                    {
                         J = -1.0 * Eigen::Vector2d(
                            0.5 * (GetPixelValue(img1, kp.pt.x + x + 1, kp.pt.y + y) -
                                   GetPixelValue(img1, kp.pt.x + x - 1, kp.pt.y + y)),
                            0.5 * (GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y + 1) -
                                   GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y - 1)));
                    }
                     b += - error *J;
                     cost += error*error;
                     if(inverse == false || iter==0)
                     {
                            H+=J*J.transpose();
                     }

                }
                
            }

            Eigen::Vector2d update =  H.ldlt().solve(b);

            if(std::isnan(update[0]))   //sometimes occurred when we have a black or white patch and H is irreversible
            {
                 std::cout<<"update is nan"<<std::endl;
                 succ = false;
                 break;
            }
            if(iter>0 && cost>lastCost)
                break;
            dx  += update[0];
            dy  += update[1];

            if(update.norm()<1e-2)
                break;

        }
        success[i] =succ;

        kp2[i].pt = kp.pt + Point2f(dx,dy);


    }
 }



 void OpticalFlowTracker::test()
 {
    std::cout<<"for test"<<std::endl;
 }