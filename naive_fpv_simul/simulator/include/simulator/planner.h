#include <ros/ros.h>
#include <future>
#include <chrono>
#include <string>
#include <thread>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

//for TakeofF(Hovering example)
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/String.h>

//For wavepoint publishing(position_publisher.cpp)
#include<simulator/center_depth.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Header.h>
#include<geometry_msgs/Pose.h>

//For defining state machine
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>

#define echo(X) std::cout<<X<<std::endl;

//Gotop1(position_publisher.cpp)
const float img_center_x=400;
const float img_center_y=400;
float x_rel,y_rel,z_rel;
float x_lab,y_lab,z_lab;
float x_drone,y_drone,z_drone;
float x_lab_initial,y_lab_initial,z_lab_initial;

//defining frame locations
std::vector<std::vector<float>> frame_coords{
                                                {8,-2},
                                                {10,10},
                                                {2,14},
                                                {-1,3}
                                                };



float iterator=0;  //iterator to count which frame the drone will pass
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
            std::cout<<"z_rel:"<<z_rel<<std::endl;

            x_lab=frame_coords[iterator][0];
            y_lab=frame_coords[iterator][1];
            z_lab=z_drone-y_rel;
}

double qx,qy,qz,qw;
void DroneRotateCallback(geometry_msgs::Pose msg){
            qx=msg.orientation.x;
            qy=msg.orientation.y;
            qz=msg.orientation.z;
            qw=msg.orientation.w;
}

std::vector<double> q_to_angles(double q0,double q1,double q2,double q3){
double roll  = atan2(2.0 * (q3 * q2 + q0 * q1) , 1.0 - 2.0 * (q1 * q1 + q2 * q2));
double pitch = asin(2.0 * (q2 * q0 - q3 * q1));
double yaw   = atan2(2.0 * (q3 * q0 + q1 * q2) , - 1.0 + 2.0 * (q0 * q0 + q1 * q1));
std::vector<double> ans;
ans.push_back(roll);
ans.push_back(pitch);
ans.push_back(yaw);
return ans;
}


namespace state_machine{

    namespace msm=boost::msm;
    namespace mpl=boost::mpl;

    //event declaration
    struct CmdTakeoff {CmdTakeoff(){}};
    struct CmdGotoP1 {CmdGotoP1(){}};
    struct CmdGotoP2 {CmdGotoP2(){}};
    struct CmdDetection {CmdDetection(){}};
    

    //defining state_machine

    struct fsm:public msm::front::state_machine_def<fsm>{

        typedef msm::active_state_switch_before_transition active_state_switch_policy;
        template<class Event,class FSM> void on_entry(Event const&,FSM &) {echo("Entered state_machine");}
        template<class Event,class FSM> void on_exit(Event const&,FSM &) {echo("Exited state_machine");}

        struct Rest:public msm::front::state<>{
            template<class Event,class FSM> void on_entry(Event const&,FSM &) {echo("Entered Rest state");}
            template<class Event,class FSM> void on_exit(Event const&,FSM &) {echo("Exited Rest state");}
        };

        struct Hover:public msm::front::state<>{
            template<class Event,class FSM> void on_entry(Event const&,FSM &) {echo("Entered Hover state");}
            template<class Event,class FSM> void on_exit(Event const&,FSM &) {echo("Exited Hover state");}
        };

        struct ReachP1:public msm::front::state<>{
            template<class Event,class FSM> void on_entry(Event const&,FSM &) {echo("Entered ReachP1 state");}
            template<class Event,class FSM> void on_exit(Event const&,FSM &) {echo("Exited ReachP1 state");}
        };

        struct ReachP2:public msm::front::state<>{
            template<class Event,class FSM> void on_entry(Event const&,FSM &) {echo("Entered ReachP2 state");}
            template<class Event,class FSM> void on_exit(Event const&,FSM &) {echo("Exited ReachP2 state");}
        };

        struct CenterDetect:public msm::front::state<>{
            template<class Event,class FSM> void on_entry(Event const&,FSM &) {echo("Entered CenterDetect state");}
            template<class Event,class FSM> void on_exit(Event const&,FSM &) {echo("Exited CenterDetect state");}
        };

        typedef Rest initial_state;

        //Takeoff Function(Mav hovering)
        void Takeoff(CmdTakeoff const &cmd){
            ros::NodeHandle nh;
            // Create a private node handle for accessing node parameters.
            ros::NodeHandle nh_private("~");
            ros::Publisher trajectory_pub =
                nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                    mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
            ROS_INFO("Started hovering example.");

            std_srvs::Empty srv;
            bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
            unsigned int i = 0;

            // Wait for 5 seconds to let the Gazebo GUI show up.
            ros::Duration(2.0).sleep();

            trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
            trajectory_msg.header.stamp = ros::Time::now();

            // Default desired position and yaw.
            Eigen::Vector3d desired_position(0.0, 0.0, 1.5);
            double desired_yaw = 0.0;

            // Overwrite defaults if set as node parameters.
            nh_private.param("x", desired_position.x(), desired_position.x());
            nh_private.param("y", desired_position.y(), desired_position.y());
            nh_private.param("z", desired_position.z(), desired_position.z());
            nh_private.param("yaw", desired_yaw, desired_yaw);

            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
                desired_position, desired_yaw, &trajectory_msg);

            ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
                    nh.getNamespace().c_str(), desired_position.x(),
                    desired_position.y(), desired_position.z());
            trajectory_pub.publish(trajectory_msg);

            return;
        }

        //Go to P1(position_publisher.cpp)
        void GotoP1(CmdGotoP1 const &cmd){

                ros::NodeHandle nh;
                ros::Rate loopRate(30);

                ros::Subscriber center_depth_sub=nh.subscribe<simulator::center_depth>("/center_depth",10,center_depthCallback);
                ros::Subscriber drone_pos_sub=nh.subscribe<geometry_msgs::PointStamped>("/iris/ground_truth/position",10,DronePositionCallback);
                ros::Subscriber drone_orient_sub=nh.subscribe<geometry_msgs::Pose>("/iris/ground_truth/pose",10,DroneRotateCallback);
                ros::Publisher wavepoint_pub=nh.advertise<geometry_msgs::PoseStamped>("/iris/command/pose",10);
                geometry_msgs::PoseStamped coordinates;

                int i=0;
                float z_avg_lab=0,x_avg_lab=0,y_avg_lab=0;
                float x_drone_initial_avg=0,y_drone_initial_avg=0,z_drone_initial_avg=0;
                int frame_rate=30;
                int z_avg_count=0;

                while(ros::ok()&&i<(frame_rate*2)){
                    ros::spinOnce();
                    x_avg_lab+=x_lab;
                    y_avg_lab+=y_lab;
                    if(isnan(z_lab)){}
                    else{
                        z_avg_lab+=z_lab;
                        z_avg_count++;
                    }
                    x_drone_initial_avg+=x_drone;
                    y_drone_initial_avg+=y_drone;
                    z_drone_initial_avg+=z_drone;
                    i++;
                    loopRate.sleep();
                }
                
                x_avg_lab=x_avg_lab/(frame_rate*2);
                y_avg_lab=y_avg_lab/(frame_rate*2);
                z_avg_lab=z_avg_lab/z_avg_count;
                x_drone_initial_avg/=(frame_rate*2);
                y_drone_initial_avg/=(frame_rate*2);
                z_drone_initial_avg/=(frame_rate*2);

                x_lab_initial=x_avg_lab;
                y_lab_initial=y_avg_lab;
                z_lab_initial=z_avg_lab+0.2;
                std::cout<<"x_lab_initial:"<<x_lab_initial<<std::endl; 
                std::cout<<"y_lab_initial:"<<y_lab_initial<<std::endl;
                std::cout<<"z_lab_initial:"<<z_lab_initial<<std::endl; 

                float unit_vec_x,unit_vec_y,unit_vec_z;
                float mag=sqrt(pow(x_lab_initial-x_drone_initial_avg,2)+pow(y_lab_initial-y_drone_initial_avg,2)+pow(z_lab_initial-z_drone_initial_avg,2));
                unit_vec_x=(x_lab_initial-x_drone_initial_avg)/mag;
                unit_vec_y=(y_lab_initial-y_drone_initial_avg)/mag;
                unit_vec_z=(z_lab_initial-z_drone_initial_avg)/mag;

                float increment_rate=0.5;
                float x_increment=unit_vec_x*increment_rate,y_increment=unit_vec_y*increment_rate,z_increment=unit_vec_z*increment_rate;
                
                i=0;
                float rel_dis=sqrt(pow(x_lab_initial-x_drone,2)+pow(y_lab_initial-y_drone,2)+pow(z_lab_initial-z_drone,2));
                std::cout<<rel_dis<<std::endl;

                while(ros::ok()&&(rel_dis>1.5)){
                    int result=ros::ok()&&(rel_dis>2);
                    std::cout<<"result:"<<result<<std::endl;
                    ros::spinOnce();
                    rel_dis=sqrt(pow(x_lab_initial-x_drone,2)+pow(y_lab_initial-y_drone,2)+pow(z_lab_initial-z_drone,2));
                    if(i==0) {
                        ROS_INFO_STREAM("Started custom wavepoint publisher");
                        i++;
                        }

                    coordinates.header=header;
                    coordinates.pose.position.x=x_drone+x_increment;
                    coordinates.pose.position.y=y_drone+y_increment;
                    coordinates.pose.position.z=z_drone+z_increment;
                    coordinates.pose.orientation.x=qx;
                    coordinates.pose.orientation.y=qy;
                    coordinates.pose.orientation.z=qz;
                    coordinates.pose.orientation.w=qw;

                    std::cout<<coordinates<<std::endl;
                    std::cout<<"rel_dis:"<<rel_dis<<std::endl;
                    
                    wavepoint_pub.publish(coordinates);
                
                    loopRate.sleep();
                }
                iterator++;
                return;
        }

        void GotoP2(CmdGotoP2 const &cmd){
                ros::NodeHandle nh;
                ros::Rate loopRate(30);

                ros::Subscriber drone_pos_sub=nh.subscribe<geometry_msgs::PointStamped>("/iris/ground_truth/position",10,DronePositionCallback);
                ros::Publisher wavepoint_pub=nh.advertise<geometry_msgs::PoseStamped>("/iris/command/pose",10);
                geometry_msgs::PoseStamped coordinates;

                float unit_vec_x,unit_vec_y,unit_vec_z;
                float mag=sqrt(pow(x_lab_initial-x_drone,2)+pow(y_lab_initial-y_drone,2)+pow(z_lab_initial-z_drone,2));
                unit_vec_x=(x_lab_initial-x_drone)/mag;
                unit_vec_y=(y_lab_initial-y_drone)/mag;
                unit_vec_z=(z_lab_initial-z_drone)/mag;

                float increment_rate=0.5;
                float x_increment=unit_vec_x*increment_rate,y_increment=unit_vec_y*increment_rate,z_increment=unit_vec_z*increment_rate;
                
                int i=0;
                float rel_dis=sqrt(pow(x_lab_initial-x_drone,2)+pow(y_lab_initial-y_drone,2)+pow(z_lab_initial-z_drone,2));

                while(ros::ok() && (rel_dis<1.7)){
                    ros::spinOnce();
                    rel_dis=sqrt(pow(x_lab_initial-x_drone,2)+pow(y_lab_initial-y_drone,2)+pow(z_lab_initial-z_drone,2));

                    coordinates.header=header;
                    coordinates.pose.position.x=x_drone+x_increment;
                    coordinates.pose.position.y=y_drone+y_increment;
                    coordinates.pose.position.z=z_drone+z_increment;
                    loopRate.sleep();
                    coordinates.pose.orientation.x=qx;
                    coordinates.pose.orientation.y=qy;
                    coordinates.pose.orientation.z=qz;
                    coordinates.pose.orientation.w=qw;

                    wavepoint_pub.publish(coordinates);

                    loopRate.sleep();

                }
                return;
        }

        void Detection(CmdDetection const &cmd){

            ros::NodeHandle nh;
            ros::Rate loopRate(30);

            ros::Publisher wavepoint_pub=nh.advertise<geometry_msgs::PoseStamped>("/iris/command/pose",10);
            ros::Subscriber drone_orient_sub=nh.subscribe<geometry_msgs::Pose>("/iris/ground_truth/pose",10,DroneRotateCallback);
            ros::Subscriber drone_pos_sub=nh.subscribe<geometry_msgs::PointStamped>("/iris/ground_truth/position",10,DronePositionCallback);
            geometry_msgs::PoseStamped coordinates;
            std::vector<double> angles=q_to_angles(qw,qx,qy,qz);
            tf2::Quaternion new_q;
            new_q.setRPY( 0+angles[0], 0+angles[1],1.57+angles[2]);           
            //new_q = new_q*q_orig;
            int i=0;
            while(ros::ok()&&i<60){
             
            i++;
            ros::spinOnce();
            coordinates.pose.position.x=x_drone;
            coordinates.pose.position.y=y_drone;
            coordinates.pose.position.z=z_drone;
            coordinates.pose.orientation.x=new_q[0];
            coordinates.pose.orientation.y=new_q[1];
            coordinates.pose.orientation.z=new_q[2];
            coordinates.pose.orientation.w=new_q[3];

            std::cout<<coordinates<<std::endl;
            
            wavepoint_pub.publish(coordinates);
            loopRate.sleep();
            }
        return;

        }



    struct transition_table : mpl::vector<
     //      Type        Start            Event            Next              Action				    Guard
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    Rest          ,  CmdTakeoff   ,  Hover         ,  &fsm::Takeoff                               >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    Hover         ,  CmdGotoP1    ,  ReachP1       ,  &fsm::GotoP1                                 >,
                          
        // // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    ReachP1       ,  CmdGotoP2     ,  ReachP2      ,  &fsm::GotoP2                                 >,
        // // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                 a_row<   ReachP2       ,  CmdDetection  ,  CenterDetect  ,  &fsm::Detection                            >,
        // // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                 a_row<   CenterDetect  ,  CmdGotoP1  ,   ReachP1         ,  &fsm::GotoP1                                >
        // // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
        >{};

    };

    typedef msm::back::state_machine<fsm> fsm_;

    static char const *const state_names[]={"Rest",
                                            "Hover",
                                            "ReachP1",
                                            "ReachP2",
                                            "CenterDetect"
    };

    void echo_state(fsm_ const& msg){echo("Current state -- " << state_names[msg.current_state()[0]]);}

    void statePublish(ros::NodeHandle nh, fsm_ *fsm)
    {
        ros::Publisher statePub = nh.advertise<std_msgs::String>("curr_state", 10);
        ros::Rate loopRate(10);

        std_msgs::String msg;
        while(ros::ok()){
            msg.data = state_names[fsm->current_state()[0]];
            statePub.publish(msg);
            loopRate.sleep();
        }
    }
}


