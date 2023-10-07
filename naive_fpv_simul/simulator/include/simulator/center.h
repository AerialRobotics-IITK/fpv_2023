  #include <ros/ros.h>
  #include <opencv2/imgproc/imgproc.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <iostream>
  #include <string>
  #include <numeric>


 std::vector<float> CenterCoordinates(cv::Mat final){

        std::vector<cv::Vec4i> linesP; // will hold the results of the detection
        cv::HoughLinesP(final, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
        
        std::vector<float> coordinates(2,0);
        if(linesP.size()==0)
        {
            cv::HoughLinesP(final, linesP, 1, CV_PI/180, 10, 10, 10 ); 
        }
        float vec_len=linesP.size();
        for( size_t i = 0; i < vec_len; i++ )
        {
            coordinates[0]+=(linesP[i])[0]+(linesP[i])[2];
            coordinates[1]+=(linesP[i])[1]+(linesP[i])[3];
        }
        
        // std::cout<<"len : "<<vec_len<<std::endl;     
        coordinates[0]=coordinates[0]/(2*vec_len);
        coordinates[1]=coordinates[1]/(2*vec_len);
        // std::cout<<"x : "<<coordinates[0]<<std::endl;
        // std::cout<<"y : "<<coordinates[1]<<std::endl;
        return coordinates;
 }