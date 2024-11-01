#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <pcl/point_types.h>  // 포인트 타입 정의
#include <pcl/point_cloud.h>  // 포인트 클라우드 클래스
#include <pcl_conversions/pcl_conversions.h>  // ros msg -> point cloud
#include <pcl/filters/passthrough.h>  // 범위 필터
#include <pcl/filters/voxel_grid.h>  // 다운샘플링 필터
#include <pcl/filters/passthrough.h>  // 범위 필터
#include "tool.h"

void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg);
double calculateAverageZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

int main(int argc,char **argv) {
    ros::init(argc, argv, "groundObstacle");
    std::cout<<ros::this_node::getName()<<std::endl;
    ros::NodeHandle nh;

    ros::Subscriber sub_scan = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 100, callback_scan);

    ros::spin();

    return 0;
}

void callback_scan(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::cout<<"CB - SCAN init"<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc_xyz);

    // 1. down sampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_vx_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(pc_xyz);
    voxel_filter.setLeafSize(0.05, 0.05, 0.05);
    voxel_filter.filter(*pc_vx_filtered);

    // 2. set ROI
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(pc_vx_filtered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-0.15, 0.15); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
    pass_x.filter(*pc_filtered_x);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(pc_filtered_x);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.1, 0.7); // 원하는 z축 범위로 설정
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass_z.filter(*pc_filtered);

    // 3. calc closest z? plane
    double mean_z = 0;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pc_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
    // min_z = tool::extractClosestPlane(pc_filtered, pc_no_ground);

    double t = msg->header.stamp.toSec();
    mean_z = calculateAverageZ(pc_filtered);

    std_msgs::Float64 output_msg;
    output_msg.data = mean_z;
    static ros::Publisher pub_obstacleDist = ros::NodeHandle().advertise<std_msgs::Float64>("/distance_obstacle", 100);
    pub_obstacleDist.publish(output_msg);

    // pack into ROS msg with avg time 
    sensor_msgs::PointCloud2 output_msg_pc;
    pcl::toROSMsg(*pc_filtered, output_msg_pc);
    output_msg_pc.header.frame_id = "camera_link";
    output_msg_pc.header.stamp = ros::Time(t);
    
    // pub them
    static ros::Publisher pub_PC = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("/base_pointcloud", 100);
    pub_PC.publish(output_msg_pc);
    std::cout<<"CB - SCAN END"<<std::endl;
}

double calculateAverageZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    double sum_z = 0.0;
    int point_count = cloud->points.size();

    for (const auto& point : cloud->points) {
        sum_z += point.z;
    }

    return (point_count > 0) ? sum_z / point_count : 0.0;
}