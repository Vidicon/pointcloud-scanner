#include <ros/ros.h>

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

bool save_once = true;

void pointcloudCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    if (save_once)
    {
        save_once = false;
        ROS_INFO("pointcloud");
        pcl::PointCloud<pcl::PointXYZI> cloud;

        // Fill in the cloud data
        cloud.width = msg->points.size();
        cloud.height = 1;
        cloud.is_dense = false;
        cloud.points.resize(cloud.width * cloud.height);
        
        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            cloud.points[i].x = msg->points[i].x;
            cloud.points[i].y = msg->points[i].y;
            cloud.points[i].z = msg->points[i].z;
            cloud.points[i].intensity = msg->channels[0].values[i];
        }

        pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_saver");
    ros::NodeHandle n;

    ros::Subscriber pointcloudSubscriber = n.subscribe("scan3d", 1000, pointcloudCallback);

    ros::Rate loop_rate(10);
    while (ros::ok() && save_once)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
