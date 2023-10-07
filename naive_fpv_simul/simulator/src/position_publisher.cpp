#include<ros/ros.h>
#include<math.h>
#include<iostream>
#include<simulator/center_depth.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Header.h>
#include<geometry_msgs/Pose.h>
using namespace std;

const float img_center_x=400;
const float img_center_y=400;
float x_rel,y_rel,z_rel;
float x_lab,y_lab,z_lab;
float x_drone,y_drone,z_drone;
std_msgs::Header header;
void DronePositionCallback(geometry_msgs::PointStamped msg){
    header=msg.header;
    x_drone=msg.point.x;
    y_drone=msg.point.y;
    z_drone=msg.point.z;
}


void center_depthCallback(simulator::center_depth msg){
    float center_x=msg.x;
    float center_y=msg.y;
    float depth=msg.depth;

    float x_angle=((center_x-img_center_x)/(2*img_center_x))*1.39;
    float y_angle=((center_y-img_center_y)/(2*img_center_y))*1.39;
    float Z=depth/(sqrt(1+tan(x_angle)*tan(x_angle)+tan(y_angle)*tan(y_angle)));
    x_rel=Z*tan(x_angle);
    y_rel=Z*tan(y_angle);
    z_rel=Z;

    x_lab=x_drone-z_rel;
    y_lab=y_drone+x_rel;
    z_lab=z_drone-y_rel;
    // cout<<"x:"<<-x_lab<<endl;
    // cout<<"y:"<<y_lab<<endl;
    // cout<<"z:"<<z_lab<<endl;
}

int main(int argc,char **argv){
    ros::init(argc,argv,"position");
    ros::NodeHandle nh;
    ros::Rate loopRate(30);

    ros::Subscriber center_depth_sub=nh.subscribe<simulator::center_depth>("/center_depth",10,center_depthCallback);
    ros::Subscriber drone_pos_sub=nh.subscribe<geometry_msgs::PointStamped>("/iris/ground_truth/position",10,DronePositionCallback);

    ros::Publisher wavepoint_pub=nh.advertise<geometry_msgs::PoseStamped>("/iris/command/pose",10);
    geometry_msgs::PoseStamped coordinates;

    int i=0;
    while(ros::ok()&&i<300){
        if(i==0) ROS_INFO_STREAM("Entered in buffer time");
        i++;
        loopRate.sleep();
    }

    ROS_INFO_STREAM("Exited the buffer time.Ready to enter vertical motion");

    i=0;
    float z_avg=0;
    while(ros::ok()&&i<30){
        ros::spinOnce();
        z_avg+=z_lab;
        i++;
        loopRate.sleep();
    }
    int frame_rate=30;
    z_avg=z_avg/frame_rate;

    float x_lab_initial=x_lab;
    float y_lab_initial=y_lab;
    float z_lab_initial=z_avg;

    i=0;

    while(ros::ok()&&i<300){
        if(i==0)   ROS_INFO_STREAM("Entered for vertical movement");
        i++;
       
        ros::spinOnce();
        coordinates.header=header;
        coordinates.pose.position.x=0;
        coordinates.pose.position.y=0;
        coordinates.pose.position.z=z_lab_initial;
        coordinates.pose.orientation.x=0;
        coordinates.pose.orientation.y=0;
        coordinates.pose.orientation.z=0;
        coordinates.pose.orientation.w=1;
        wavepoint_pub.publish(coordinates);
        cout<<coordinates.pose.position<<endl;
        loopRate.sleep();
    }

    ROS_INFO_STREAM("Exited the vertical motion");
   
    
    i=0;
    int flag=1;
    while(ros::ok()){
        ros::spinOnce();
        if(i==0) {
            ROS_INFO_STREAM("Started custom wavepoint publisher");
            i++;
            }
        cout<<"x_rel:"<<abs(z_rel)<<endl;
        float x_temp=0.01;
        if(flag){
            if(abs(z_rel)>2) {
                if(z_rel<=2) flag=0;
                coordinates.pose.position.x=x_drone+x_temp;
            }
        }
        if(flag==0) {
            coordinates.pose.position.x=x_drone+x_temp;
        }
        
        coordinates.header=header;
        coordinates.pose.position.y=-y_lab_initial;
        coordinates.pose.position.z=z_lab_initial;
        // coordinates.pose.orientation.x=0;
        // coordinates.pose.orientation.y=0;
        // coordinates.pose.orientation.z=0;
        // coordinates.pose.orientation.w=1;

        cout<<coordinates.pose.position<<endl;
        
        wavepoint_pub.publish(coordinates);
       
        loopRate.sleep();

    }

    return 0;
}