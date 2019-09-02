#include <ros/ros.h>

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"

#define STEPPER_ANGLE 1.8

enum Microstep
{
    FULL = 1,
    HALF = 2,
    QUARTER = 4,
    EIGHTH = 8,
    SIXTEENTH = 16
};

ros::Publisher g_pointcloud_pub;
ros::Publisher g_steps_pub;

int g_currentSteps = 0;
bool scanEnable = false;
bool liveUpdateEnable = true;
int g_seq = 0;

sensor_msgs::ChannelFloat32 g_channelIntensity;
std::vector<geometry_msgs::Point32> g_pointcloudPoints;

void sendPointCloud()
{
    sensor_msgs::PointCloud pointcloudMsg;
    pointcloudMsg.header.seq = g_seq++;
    pointcloudMsg.header.stamp = ros::Time::now();
    pointcloudMsg.header.frame_id = "world";

    pointcloudMsg.points = g_pointcloudPoints;
    pointcloudMsg.channels.clear();
    pointcloudMsg.channels.push_back(g_channelIntensity);

    g_pointcloud_pub.publish(pointcloudMsg);
}

geometry_msgs::Point32 angularDistanceToPoint(float angle, float rotation, float distance)
{
    geometry_msgs::Point32 point;
    point.x = (sin(rotation) * sin(angle) * distance);
    point.y = (cos(rotation) * sin(angle) * distance);
    point.z = (cos(angle) * distance);
    return point;
}

void addScanToPointcloud(float rotation, sensor_msgs::LaserScan scan)
{
    uint16_t pointCount = -1;
    for (float i = scan.angle_min; i < scan.angle_max; i += scan.angle_increment)
    {
        pointCount++; // 0 to 359

        if ((scan.ranges[pointCount] < 0.08) || (scan.ranges[pointCount] > 5.0)) // only use scan data more than 10 cm and 5 m. only needed because my lidar node is bad.
            continue;

        g_pointcloudPoints.push_back(angularDistanceToPoint(i, rotation, scan.ranges[pointCount])); //calculede 3d point from tilt angle, pan angle and distance.
        g_channelIntensity.values.push_back(scan.intensities[pointCount]);
    }
}

float stepsToAngle(int steps, Microstep microSteping)
{
    return steps * (STEPPER_ANGLE / microSteping);
}

void sendSteps(int angle)
{
    std_msgs::UInt16 stepsMsg;
    stepsMsg.data = angle;
    g_steps_pub.publish(stepsMsg);
}

void stepsCallback(const std_msgs::UInt16::ConstPtr &msg)
{
    g_currentSteps = msg->data;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    if (!scanEnable)
    {
        return;
    }

    addScanToPointcloud(0.0, *scan);
    if (liveUpdateEnable)
        sendPointCloud();
}

bool ScanStart(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if (!scanEnable)
    {
        scanEnable = true;
        res.success = true;
        res.message = "scan started";
    }
    else
    {
        res.success = false;
        res.message = "scan already started!";
    }
    return true;
}

bool ScanPub(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    sendPointCloud();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_combiner");
    ros::NodeHandle n;
    ros::Subscriber scanSubscriber = n.subscribe("scan", 1000, scanCallback);
    g_pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("pointcloud", 1000);
    g_steps_pub = n.advertise<std_msgs::UInt16>("steps", 1000);
    ros::Subscriber stepsSubscriber = n.subscribe("currentSteps", 1000, stepsCallback);
    ros::ServiceServer service = n.advertiseService("scan/start", ScanStart);
    ros::ServiceServer service = n.advertiseService("scan/pub", ScanPub);
}