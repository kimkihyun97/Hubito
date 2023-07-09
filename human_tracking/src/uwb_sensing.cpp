#include "uwb_sensing.h"

namespace UWBsensorteam{

    UWBDataHandler::UWBDataHandler()
    : nh_(),
      object_sub_(nh_.subscribe("objects_position/message", 1, &UWBDataHandler::object0Callback, this)),
      scan_sub_(nh_.subscribe("scan", 1, &UWBDataHandler::scan0Callback, this)),
      imu_sub_(new message_filters::Subscriber<sensor_msgs::Imu>(nh_, "camera/imu", 1)),
      uwb_sub_(new message_filters::Subscriber<human_tracking::UWB>(nh_, "UWB", 1)),
      sync_(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(1), *imu_sub_, *uwb_sub_)),
      control_pub_(nh_.advertise<std_msgs::Int16MultiArray>("control_command", 5)),
      filtered_data_pub_(nh_.advertise<human_tracking::filtered_data>("filtered_data", 5)),
      pre_velocity(0.0), pre_angular_velocity(0.0),
      pre_rad_vel(0.0),pre_left_pwm(0), pre_right_pwm(0),pre_time(0),
      robot_rad_imu(0)
    {
        sync_->registerCallback(boost::bind(&UWBDataHandler::control0callback, this, _1, _2));

    }

    void UWBDataHandler::object0Callback(const human_tracking::ObjectPositionsConstPtr& objects)
    {
      if(!objects->object_positions.empty() && !objects->object_positions[0].z == 0 && !objects->object_positions[0].x == 0)
      {
        object_active = true;
        object.clear();
        object.push_back(objects->object_positions[0].z);
        object.push_back(objects->object_positions[0].x);
      }
    }
    
    void UWBDataHandler::scan0Callback(const sensor_msgs::LaserScanConstPtr& scan)
    { 
      //detecting human based on uwb angle tolerance ~10 ~ 10 degree
      if(uwb_active)
      {
        std::vector<float> angle_values;
        std::vector<float> range_values;

        float angle_min = -(scan->angle_min);
        float angle_increment = scan->angle_increment;
      
        for (int i = 0; i < scan->ranges.size(); ++i)
        {
          float current_angle = angle_min - i * angle_increment;
          if (std::abs(current_angle - angle_measure) <= angle_tolerance && !std::isnan(scan->ranges[i]))
          {
            angle_values.push_back(current_angle);
            range_values.push_back(scan->ranges[i]);
          }
        }

         scan_dis = std::accumulate(range_values.begin(), range_values.end(), 0.0) / range_values.size();
         scan_angle = std::accumulate(angle_values.begin(), angle_values.end(), 0.0) / angle_values.size();
 
         ROS_INFO("Average range: %f", scan_dis);
         ROS_INFO("Average angle: %f", scan_angle);
         scan_active = true;

         // 추후에 장애물 관련 코드 추가 예정
      }
    }

    void UWBDataHandler::control0callback(const sensor_msgs::ImuConstPtr& imu, const UWBConstPtr& uwb)
    {       
      // time
      if(pre_time.toSec() == 0)
      {
         pre_time = ros::Time::now();
         cur_time = pre_time;
      }
      else
      {
         cur_time = ros::Time::now();
      }

      dt = (cur_time - pre_time).toSec();
      
      // uwb info
      dis_measure = uwb->dis;
      angle_measure = (uwb->angle);
      uwb_active = true;
      ROS_INFO("dis_measure : %f",dis_measure);
      ROS_INFO("angle_measure : %f",angle_measure);
      
      // extended kalman-filter -> 작동이 잘되지 않아서 검즘이 필요
      // control input
      ekf_.u << pre_velocity, pre_angular_velocity;
      ekf_.predict(dt);
      
      sensor0fusion(imu);

      ekf_.update(human.z, human.x);

      //publish filtered datas
      dataset.rz = ekf_.x(0);
      dataset.rx = ekf_.x(1);
      dataset.ro = ekf_.x(2);
      update0humanpose();

      dataset.dis = sqrt(pow(dataset.hz - dataset.rz, 2) + pow(dataset.hx - dataset.rx,2));
      dataset.angle = atan2(dataset.hx - dataset.rx,dataset.hz - dataset.rz) * RAD_TO_DEG;
      filtered_data_pub_.publish(dataset);
      
     
     ROS_INFO("robot_global_pose : %f,%f, %f",dataset.rz, dataset.rx, dataset.ro);

     // dis status
      if(dis_measure < dis_target_range)                               
      {
          stop_go_flag=1;                                          
      }
      if(stop_go_flag == 1)                                       
      {
          if(dis_measure > dis_target_range + dis_dead_zone_range)                         
          {
              stop_go_flag=0;                                        
          }
      }
      
      // angle status
      if(std::abs(angle_measure) < angle_target_range)                               
      {
          stop_rotation_flag=1;                                        
      }
      if(stop_rotation_flag == 1)                                       
      {
          if(std::abs(angle_measure) > angle_dead_zone_range)                         
          {
              stop_rotation_flag=0;                                        
          }
      }

      //control
      if(stop_go_flag == 1)
      {
        TRACKING_VELOCITY = 0.0;
        if(stop_rotation_flag ==1)
        {
          CONTROL_ANGLE = 0.0;
        }
        else
        {
          CONTROL_ANGLE = -((PID_control(angle_target, angle_measure, &ANGLE_PID)) / RAD_TO_DEG);
        }
      }
      else
      {
        if(stop_rotation_flag ==1)
        {
          CONTROL_ANGLE = 0.0;
        }
        else
        {
          CONTROL_ANGLE = -((PID_control(angle_target, angle_measure, &ANGLE_PID)) / RAD_TO_DEG);
        }
        TRACKING_VELOCITY = -PID_control(dis_target_range, dis_measure, &DIS_PID);
      }
    
    
    ROS_INFO("tracking & control angle : %f, %f",TRACKING_VELOCITY, CONTROL_ANGLE);

    vel_to_motor(TRACKING_VELOCITY, CONTROL_ANGLE);

    pre_velocity = ((left_vel + right_vel) / 2);
    pre_angular_velocity = ((right_vel - left_vel) / 0.246);
    uwb_active = false; 
    }
    
    // 수정 및 추가적인 작업이 필요합니다.
    // for measurement matrix z
    void UWBDataHandler::sensor0fusion(const sensor_msgs::ImuConstPtr& imu)
    {
      if(object_active)
      {
        ROS_INFO("Object active");
        human.z = object[0];
        human.x = object[1];
        object_active = false;
      }
      else if(scan_active)
      {
        human.z = scan_dis * cos(scan_angle);
        human.x = scan_dis * sin(scan_angle);
      }
      else
      {
        human.z = dis_measure * cos(angle_measure / RAD_TO_DEG);
        human.x = dis_measure * sin(angle_measure / RAD_TO_DEG);
      }
      
      float rad_vel = -(imu->angular_velocity.y);

      float sensor_dis = dis_measure;
      float sensor_angle = angle_measure / RAD_TO_DEG;
      robot_rad_imu += ((pre_rad_vel + rad_vel) / 2) * dt;
      if (robot_rad_imu > 2 * M_PI) 
      {
        robot_rad_imu -= 2.0 * M_PI;
      } 
      else if (robot_rad_imu < -2 * M_PI) 
      {
        robot_rad_imu += 2.0 * M_PI;
      }
      ekf_.z << sensor_dis, sensor_angle, robot_rad_imu;

      //update
      pre_rad_vel = rad_vel;
    }
    // 수정 및 추가적인 작업이 필요합니다.
    // human position based on updated robot pose
    void UWBDataHandler::update0humanpose()
    {
      float uwb_z = ekf_.x(0) + dis_measure * cos(angle_measure / RAD_TO_DEG);
      float uwb_x = ekf_.x(1) + dis_measure * sin(angle_measure / RAD_TO_DEG);
      // 가중평균을 통해 위치 추정, 수정 및 보완 필요
      dataset.hz = 0.6 * uwb_z + 0.4 * (ekf_.x(0) + human.z);
      dataset.hx = 0.6 * uwb_z + 0.4 * (ekf_.x(1) + human.x);
    }
    void UWBDataHandler::vel_to_motor(const float& linear_vel, const float& angular_vel)
    {
      control_command.data.clear();
      int16_t cur_left_pwm, cur_right_pwm;
      // divide vel
      left_vel = ((linear_vel - (angular_vel * 0.246)/2));
      right_vel = ((linear_vel + (angular_vel * 0.246)/2));
      
      left_pwm = (int16_t)(left_vel * VEL_TO_PWM);
      right_pwm = (int16_t)(right_vel * VEL_TO_PWM);
      

      if(left_pwm < -255)
      {
        left_pwm = -255;
      }
      if(left_pwm > 255)
      {
        left_pwm = 255;
      }

      if(right_pwm < -255)
      {
        right_pwm = -255;
      }
      if(right_pwm > 255)
      {
        right_pwm = 255;
      }

      cur_left_pwm = pre_left_pwm + (int16_t)PID_control(pre_left_pwm, left_pwm, &Left_PID);
      cur_right_pwm = pre_right_pwm + (int16_t)PID_control(pre_right_pwm, right_pwm, &right_PID);
      ROS_INFO("publish left, right pwm : %d, %d",cur_left_pwm, cur_right_pwm);
    
      control_command.data = {cur_left_pwm, cur_right_pwm};
      
      control_pub_.publish(control_command);



      left_vel = (float)(cur_left_pwm / VEL_TO_PWM);
      right_vel = (float)(cur_right_pwm / VEL_TO_PWM);

      pre_left_pwm = cur_left_pwm;
      pre_right_pwm = cur_right_pwm;
    }

    UWBDataHandler::~UWBDataHandler()
    {
      delete imu_sub_;
      delete uwb_sub_;
      delete sync_;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uwb_sensing");
    UWBsensorteam::UWBDataHandler UWBDataHandler;
    ros::spin();
    return 0;
}