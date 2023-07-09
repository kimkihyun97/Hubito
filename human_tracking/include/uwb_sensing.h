#ifndef HUMAN_TRACKING
#define HUMAN_TRACKING

#include <nlink_parser/LinktrackAoaNodeframe0.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Int16MultiArray.h>
#include "human_tracking/UWB.h"
#include "human_tracking/filtered_data.h"
#include "human_tracking/ObjectPosition.h"
#include "human_tracking/ObjectPositions.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "pid.h"
#include "EKF.h"
#include <algorithm>
#include <vector>
#include <cmath>
#include <utility>
#include <numeric>

namespace UWBsensorteam {
    typedef nlink_parser::LinktrackAoaNodeframe0 UWBdata;
    typedef boost::shared_ptr< ::human_tracking::UWB const> UWBConstPtr;
    typedef boost::shared_ptr< ::human_tracking::ObjectPositions const> ObjectPositionsConstPtr;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, human_tracking::UWB> MySyncPolicy;
    
    struct human_pose
    {
      float z, x;
      human_pose() : z(0.0), x(0.0) {}
    };

    double dt;

    class UWBDataHandler 
    {
      public:
        UWBDataHandler();
        virtual ~UWBDataHandler();
        const double RAD_TO_DEG = 180/M_PI;
        const float VEL_TO_PWM = 1 / 0.00232;
        ros::Time pre_time, cur_time;
      private :
        ros::NodeHandle nh_;
        ros::Subscriber object_sub_;
        ros::Subscriber scan_sub_;
        ros::Publisher control_pub_;
        ros::Publisher robot_pose_pub_;
        ros::Publisher human_pose_pub_;
        ros::Publisher filtered_data_pub_;
        message_filters::Subscriber<sensor_msgs::Imu>* imu_sub_;
        message_filters::Subscriber<human_tracking::UWB>* uwb_sub_;
        message_filters::Synchronizer<MySyncPolicy>* sync_;
        void object0Callback(const human_tracking::ObjectPositionsConstPtr& objects);
        void scan0Callback(const sensor_msgs::LaserScanConstPtr& scan);
        void control0callback(const sensor_msgs::ImuConstPtr& imu, const UWBConstPtr& uwb);
        void sensor0fusion(const sensor_msgs::ImuConstPtr& imu);
        void update0humanpose();
        void vel_to_motor(const float& linear_vel, const float& angular_vel);

        EKF ekf_;
        human_pose human;

        float TRACKING_VELOCITY;
        float CONTROL_ANGLE;
        int LEFT_VEL;
        int RIGHT_VEL;
        uint8_t object_active;
        uint8_t scan_active = false;
        uint8_t uwb_active = false;
        uint8_t obstacle_active = false;
        std::vector<float> object;
        
        // for scan
        float scan_dis, scan_angle;
        float angle_tolerance = 0.174533; // 10 degree

        // for odom
        float left_odom, right_odom;

        // for kalman
        float pre_rad_vel;
        float robot_rad_imu;
        float pre_velocity, pre_angular_velocity;
        float calibration_x = 0.13;
        float calibration_z = 1.0;
        human_tracking::filtered_data dataset;

        // for uwb
        float dis_measure;
        float angle_measure;
        float angle_target_range=5.0;            
	      float angle_dead_zone_range=10.0;         
	      uint8_t stop_go_flag=0;
        uint8_t stop_rotation_flag=0;                     
	      float angle_target=0.0;                                                      
	      float dis_target_range=0.8;
        float dis_dead_zone_range = 0.2;

        // for control
        std_msgs::Int16MultiArray control_command;
        float left_vel, right_vel;
        int16_t left_pwm, right_pwm, pre_left_pwm, pre_right_pwm;
    };
};

#endif