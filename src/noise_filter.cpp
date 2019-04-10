#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <queue>
#include <vector>
#include <boost/foreach.hpp>

class noiseFilter{
    public:
        noiseFilter():
            nh_()    
        {
            isFull=false;
            count=0;
            nh_.param<std::string>("/name_input",name_input,"/sick_visionary_t_driver/laserscan");
            nh_.param<std::string>("/name_output",name_output,"/sick_visionary_t_driver/laserscan_filtered");
            nh_.param("/queue",queue, 5);
            laser_input = nh_.subscribe(name_input, 10, &noiseFilter::Callback, this);
            laser_output = nh_.advertise<sensor_msgs::LaserScan>(name_output, 10);
        }


    private:
        ros::NodeHandle nh_;
        ros::Subscriber laser_input;
        ros::Publisher laser_output;
        std::queue<sensor_msgs::LaserScan> buffe;
        //sensor_msgs::LaserScan laser_old;
        sensor_msgs::LaserScan laser_msg;
        bool isFull;
        std::string name_input;
        std::string name_output;
        double limit;
        int count;
        int queue;


        void Callback(const sensor_msgs::LaserScan& msg){
            if(isFull==false){
                buffe.push(msg);
                //laser_old=msg;
                laser_output.publish(msg);
                count++;
                if(count==(queue-1))
                    isFull=true;
                return ;
            }
            else{
                buffe.push(msg);
                laser_msg.header=msg.header;
                laser_msg.angle_min=msg.angle_min;
                laser_msg.angle_max=msg.angle_max;
                laser_msg.angle_increment=msg.angle_increment;
                laser_msg.time_increment=msg.time_increment;
                laser_msg.scan_time=msg.scan_time;
                laser_msg.range_min=msg.range_min;
                laser_msg.range_max=msg.range_max;
                laser_msg.ranges.resize(msg.ranges.size());
                std::vector<sensor_msgs::LaserScan> temp;
                    for(int i=0;i<queue;++i){
                        temp.push_back(buffe.front());buffe.pop();
                    }
                    for(int i=1;i<queue;i++){
                        buffe.push(temp[i]);
                    }
                for (int i=0;i<msg.ranges.size();++i){
                    //double percentage=(msg.ranges[i]-laser_old.ranges[i])/laser_old.ranges[i];
                    //ROS_INFO_STREAM("current error between two adjacent point is "<<percentage);
                    //nh_.param("/limit",limit,0.2);
                    //if(percentage<-1*limit){
                    //    laser_msg.ranges[i]=laser_old.ranges[i];
                    //}
                    //else{
                    //    laser_msg.ranges[i]=msg.ranges[i];
                    //}
                    float max=0;
                    BOOST_FOREACH(sensor_msgs::LaserScan& laser, temp)
                    {
                        if(laser.ranges[i]>max)
                            max=laser.ranges[i];
                    }
                    laser_msg.ranges[i]=max;
                }
                laser_output.publish(laser_msg);
               
            }
            
            //laser_old=msg;
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
