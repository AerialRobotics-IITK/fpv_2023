  #include <ros/ros.h>
  #include <opencv2/imgproc/imgproc.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <iostream>
  #include <string>
  #include <numeric>
 cv::Mat FinalContour(cv::Mat img){
    
       //Image Manipulation logic
         int lowThreshold = 80;
        const int max_lowThreshold = 200;
        const int ratio = 3;
        const int kernel_size = 3;
        const double scale=1;
        const char* window_name = "Edge Map";
        const int corner_threshold=100;
        const double epsilon=1;
        cv::resize(img, img, cv::Size(img.cols*scale, img.rows*scale));
        
        cv::Mat hsv;
        cv::cvtColor(img,hsv,CV_BGR2HSV);

        std::vector<cv::Mat> channels;
        cv::split(hsv, channels);

        cv::Mat H = channels[0];
        cv::Mat S = channels[1];
        cv::Mat V = channels[2];
        cv::GaussianBlur(S, S, cv::Size(15,15), 0, 0);
        cv::Mat grey,dst,detected_edges;
        cv::cvtColor(img, grey, CV_BGR2GRAY);
        dst.create( img.size(), img.type() );
        
        cv::Mat canny_output;
        cv::Canny( S, canny_output, lowThreshold, lowThreshold*ratio, kernel_size );
        cv::GaussianBlur(canny_output, canny_output, cv::Size(3,3), 0, 0);

        
        cv::Mat final;

        std::vector< std::vector <cv::Point> > contours; // Vector for storing contour

        std::vector< cv::Vec4i > hierarchy;
        cv::findContours( canny_output, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
        if(contours.size()==0)
        {
            cv::threshold(S,S,20,255,0);
            cv::Canny(S,canny_output,lowThreshold,lowThreshold*ratio,kernel_size);
            cv::findContours( canny_output, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
        }
        
        std::vector< std::vector<cv::Point> > hull(contours.size());
        for(int i = 0; i < contours.size(); i++)
            cv::convexHull(cv::Mat(contours[i]), hull[i], false);
        double maxArea=0;
        double maxAreaContourId = 0;
        for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
        {
            double newArea=cv::contourArea(contours[i]);
            double convexArea=cv::contourArea(hull[i]);
            if(convexArea/newArea>1.2)
                continue;
            if (newArea > maxArea) {
                maxArea = newArea;
                maxAreaContourId = i;
            } 
        }

        final.create( S.size(), S.type() );
        final = cv::Scalar::all(0);
        cv::drawContours(final, contours, maxAreaContourId, 255, -1);

        return final;
 }