#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include "datapoint/dataPoint.h"
#include "validate/headers/boxValidator.h"
#include "analyze/headers/basicAnalyser.h"

sensor_msgs::PointCloud2 pointcloud_valid;
ros::Publisher points_pub;
ros::Publisher risk_pub;

BoxValidator bbVal;
BasicAnalyzer an;

void onLidarData(const sensor_msgs::PointCloud2 pointcloud)
{
    // Filter out all points not on route
    bbVal.Validate(pointcloud, pointcloud_valid);

    // Calculate risk from validated points
    float risk = an.Analyze(pointcloud_valid);
    std_msgs::Float32 msg;
    msg.data = risk;

    points_pub.publish(pointcloud_valid);
    risk_pub.publish(msg);
}

std::vector<float> convertToArray(std::string value)
{
    std::stringstream ss(value);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> tokens(begin, end);
    std::vector<float> floats;

    for (auto &s: tokens) {
        floats.push_back(std::stof(s));
    }
 
    return floats;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "lidarcontrol");
    ros::NodeHandle nh;
    
    std::string validatorConfigParam, analyzerConfigParam;

    ros::param::get("~lidarcontrol/validator_config", validatorConfigParam);
    ros::param::get("~lidarcontrol/analyzer_config", analyzerConfigParam);    

    points_pub = nh.advertise<sensor_msgs::PointCloud2>("validated_points", 1000);
    risk_pub = nh.advertise<std_msgs::Float32>("lidar_calculated_risk", 1000);

    ros::Subscriber rcin_sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 10, onLidarData);

    bbVal.Configure(convertToArray(validatorConfigParam));
    an.Configure(convertToArray(analyzerConfigParam));

    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}