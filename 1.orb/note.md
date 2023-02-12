## 手写ORB特征(14讲)

* 计算程序运行时间
    ```c++
    chrono:steady_clock::time_point t1 = chrono::steady_clock::now();
    chrono:steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double>time_used =                        chrono::duration_cast<chrono:duration<double>>(t2-t1);
    cout<<time_used.count();

    ```

* opencv特征点检测过程
    ```c++
    Ptr<FeatureDetector>detector = ORB::create();      
    Ptr<DescriptorExtractor>descriptor = ORB::create();  
    Ptr<DescriptorMatcher>mathcer = DescriptorMatcher::create("BruteForce-Hamming"); 
    ```

* drawMatches  和 drawKeyPoint  feature2D 里边提供的函数 可以可视化特征点提取效果和检测效果
* **Fast角点的尺度不变性靠的是特征金字塔，旋转不变性靠的是灰度质心法 ，称为Oriented Fast**
* ORB提取特征+特征匹配，大部分时间花费再特征提取上

* _mm_popcnt_u32(descriptor1[i1][k]^descriptor2[i2][k]); 达到计算汉明距离的作用，SSE指令集  #include <nmmintrin.h>

* **描述子的角度方向：正常角度方向是$\arctan \frac{m01}{m10}$  ，但是我得到sin  和 cos  一样可以用旋转公式取算得对应BRIEF旋转方向，sin cos 不用掉函数，对比邻这种算**



