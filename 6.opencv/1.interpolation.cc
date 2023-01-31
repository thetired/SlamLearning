// reference  
//https://blog.csdn.net/qq_37541097/article/details/112564822?ops_request_misc=%7B%22request%5Fid%22%3A%22167513583116800222861035%22%2C%22scm%22%3A%2220140713.130102334.pc%5Fblog.%22%7D&request_id=167513583116800222861035&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~blog~first_rank_ecpm_v1~rank_v31_ecpm-1-112564822-null-null.blog_rank_default&utm_term=%E6%8F%92%E5%80%BC&spm=1018.2226.3001.4450


#include <iostream>
#include "opencv2/core.hpp"
#include"opencv2/highgui.hpp"

int  GetPixelValue(const cv::Mat &img, float x, float y)
{
    if(x<0 || y<0 || x>=img.cols-1 || y>=img.rows-1)
        return 0;
    float xx = x- floor(x);
    float yy = y-floor(y);
    int x_a1 = std::min(img.cols-1,int(x)+1);
    int y_a1 = std::min(img.rows-1,int(y)+1);

    return int((1 - xx) * (1 - yy) * img.at<uchar>(y, x)
                        + xx * (1 - yy) * img.at<uchar>(y, x_a1)
                        + (1 - xx) * yy * img.at<uchar>(y_a1, x)
                        + xx * yy * img.at<uchar>(y_a1, x_a1));
}

int main(int argc, char** argv)
{
        cv::Mat img  = cv::imread("../ubuntu.png");
      
       //  -------------fot test

        cv:imshow("test",img);
        cv::waitKey(0);
}