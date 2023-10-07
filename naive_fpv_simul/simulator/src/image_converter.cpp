    #include <ros/ros.h>
    #include <image_transport/image_transport.h>
    #include <cv_bridge/cv_bridge.h>
    #include <sensor_msgs/image_encodings.h>
    #include <simulator/center_depth.h>
    #include <opencv2/imgproc/imgproc.hpp>
    #include <opencv2/highgui/highgui.hpp>
    #include "../include/simulator/final_contour.h"
    #include "../include/simulator/center.h"
    #include "../include/simulator/depth.h"
    
   static const std::string OPENCV_WINDOW = "Image window";

   class ImageConverter
   {
     ros::NodeHandle nh_;
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_;
     image_transport::Publisher image_pub_;
     ros::Publisher center_depth=nh_.advertise<simulator::center_depth>("/center_depth",10);
   
   public:
     ImageConverter()
       : it_(nh_) 
     {
       // Subscrive to input video feed and publish output video feed
       image_sub_ = it_.subscribe("/iris/camera_red_iris/image_raw", 1,
         &ImageConverter::imageCb, this);
       image_pub_ = it_.advertise("/image_converter/output_video", 1);
   
      //  cv::namedWindow(OPENCV_WINDOW);
     }
   
     ~ImageConverter()
     {
       cv::destroyWindow(OPENCV_WINDOW);
     }
   
     void imageCb(const sensor_msgs::ImageConstPtr& msg)
     {
       cv_bridge::CvImagePtr cv_ptr;
       try
       {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
       }
   
        // Update GUI Window
        cv::Mat img=cv_ptr->image;
        // cv::imshow(OPENCV_WINDOW, img);

        cv::Mat final=FinalContour(img);   //Center is a function defined in header file final_contour.h  //getting fianl Contour selected

        std::vector<float> coordinates= CenterCoordinates(final);//getting coordiantes of center of frame
        
        double depth=Depth(final);
        // std::cout<<"depth:"<<depth<<std::endl;
        
        cv::circle( final,cv::Point(coordinates[0],coordinates[1]), 5,  cv::Scalar(0), 2, 8, 0 );
        //cv::imshow("Center",final);  //final image which contains rhe center
        cv::waitKey(3);
   
       // Output modified video stream
       cv_bridge::CvImage out_msg;
       out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
       out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; // Or whatever    sensor_msgs::image_encodings::TYPE_32FC1;
       out_msg.image    = final; // Your cv::Mat


      //Center depth publisher
      simulator::center_depth center_depth_msg;
      center_depth_msg.x=coordinates[0];
      center_depth_msg.y=coordinates[1];
      center_depth_msg.depth=depth;
      center_depth.publish(center_depth_msg);

      image_pub_.publish(out_msg.toImageMsg());
     
     }
   };
   
   
   int main(int argc, char** argv)
   {
        ros::init(argc, argv, "image_converter");
        ImageConverter ic;
        ros::spin();

        return 0;
   }