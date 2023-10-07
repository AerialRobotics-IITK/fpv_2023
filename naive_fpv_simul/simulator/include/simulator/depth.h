  #include <ros/ros.h>
  #include <opencv2/imgproc/imgproc.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <iostream>
  #include <string>
  #include <numeric>


  double Depth(cv::Mat final){
    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
    cv::HoughLinesP(final, linesP, 1, CV_PI/180, 200, 50, 10 ); // runs the actual detection
    const double ref_dist=1;
    const double ref_size=58.41;
     
    if(linesP.size()==0)
    {
        cv::HoughLinesP(final, linesP, 1, CV_PI/180, 10, 10, 10 ); 
    }
    int vec_len=linesP.size();
    double depth=0;
    for( size_t i = 0; i < vec_len; i++ )
    {   
        double x1=linesP[i][0];
        double y1=linesP[i][1];
        double x2=linesP[i][2];
        double y2=linesP[i][3];
        double d=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
        d=(ref_size/d)*ref_dist;
        depth+=d;
    }
    
    depth=depth/(vec_len);
    depth=depth*10;
    // std::cout<<"depth:"<<depth<<std::endl;

    return depth;
  }