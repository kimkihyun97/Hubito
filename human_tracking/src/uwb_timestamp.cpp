#include "uwb_sensing.h"

ros::Publisher pub;
ros::Subscriber sub;
void timeadd0callback(const nlink_parser::LinktrackAoaNodeframe0& uwb)
{
    if(uwb.nodes.empty()) 
    {
        ROS_WARN("UWB nodes is empty.");
    }
    else
    {
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        human_tracking::UWB custom_msg; 

        custom_msg.header = header;
        custom_msg.dis = uwb.nodes[0].dis;
        custom_msg.angle = uwb.nodes[0].angle;

        
        pub.publish(custom_msg);

    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uwb_timestamp");
    ros::NodeHandle nh; 
    
    pub = nh.advertise<human_tracking::UWB>("UWB", 1000);
    sub = nh.subscribe("nlink_linktrack_aoa_nodeframe0", 1000, timeadd0callback);

    ros::spin();

    return 0;
}
