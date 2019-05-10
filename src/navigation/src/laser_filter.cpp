#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    static ros::NodeHandle n;
	static ros::Publisher filtered_laser_pub = n.advertise<sensor_msgs::LaserScan>("/scan", 50);
    sensor_msgs::LaserScan newScan(*msg);
    for(size_t i = 0; i < newScan.ranges.size(); i++)
    {
        if (newScan.ranges[i] < 0.0011)
        {
            newScan.ranges[i] = 31.0;
        }
    }
    filtered_laser_pub.publish(newScan);
}

int main(int argc, char **agrv)
{
    ros::init(argc, agrv, "map_base_tf_publisher");
    ros::NodeHandle nh;
    auto laser_sub = nh.subscribe("raw_scan", 50, laser_callback);
    ros::spin();
}