#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class noiseFilter{
    public:
        noiseFilter():
            nh_()    
        {
            isFirst=true;
            nh_.param<std::string>("/name_input",name_input,"/sick_visionary_t_driver/laserscan");
            nh_.param<std::string>("/name_output",name_output,"/sick_visionary_t_driver/laserscan_filtered");
            laser_input = nh_.subscribe(name_input, 1, &noiseFilter::Callback, this);
            laser_output = nh_.advertise<sensor_msgs::LaserScan>(name_output, 1);
        }


    private:
        ros::NodeHandle nh_;
        ros::Subscriber laser_input;
        ros::Publisher laser_output;
         sensor_msgs::LaserScan laser_old;
         sensor_msgs::LaserScan laser_msg;
        bool isFirst;
        std::string name_input;
        std::string name_output;
        double limit;

        void Callback(const sensor_msgs::LaserScan& msg){
            if(isFirst==true){
                laser_old=msg;
                laser_output.publish(msg);
                isFirst=false;
                return ;
            }
            else{
                laser_msg.header=msg.header;
                laser_msg.angle_min=msg.angle_min;
                laser_msg.angle_max=msg.angle_max;
                laser_msg.angle_increment=msg.angle_increment;
                laser_msg.time_increment=msg.time_increment;
                laser_msg.scan_time=msg.scan_time;
                laser_msg.range_min=msg.range_min;
                laser_msg.range_max=msg.range_max;
                laser_msg.ranges.resize(msg.ranges.size());
                for (int i=0;i<msg.ranges.size();++i){
                    double percentage=(msg.ranges[i]-laser_old.ranges[i])/laser_old.ranges[i];
                    ROS_INFO_STREAM("current error between two adjacent point is "<<percentage);
                    nh_.param("limit",limit,0.2);
                    if(percentage<-1*limit){
                        laser_msg.ranges[i]=laser_old.ranges[i];
                    }
                    else{
                        laser_msg.ranges[i]=msg.ranges[i];
                    }
                }
                laser_output.publish(laser_msg);
            }
            
            laser_old=msg;
            return ;
        }


};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "noise_filter");

    noiseFilter filter;
    ros::spin();
    return 0;
}
