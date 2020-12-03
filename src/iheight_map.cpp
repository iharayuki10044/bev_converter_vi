#include "iheight_map/iheight_map.h"

IheightMap::IheightMap(void)
: nh("~")
{
    nh.param("Hz", Hz, {10});
    nh.param("HEIGHT_THRESHOLD", HEIGHT_THRESHOLD, {0.25});
    // 0.4 ball vanish

    pc_subscriber = nh.subscribe("/velodyne_points",10, &IheightMap::pc_callback, this);
    pc_publisher = nh.advertise<sensor_msgs::PointCloud2>("velodyne_obstacles", 1);
}

void IheightMap::pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 input_pc;
    input_pc = *msg;
    pcl::fromROSMsg(input_pc, *pcl_input_pc);
    pcl_output_pc->clear();
    pcl_output_pc->header = pcl_input_pc->header;
    pcl_output_pc->header.frame_id ="base_footprint";
    // pcl_output_pc->header = input_pc->header;

    for(int i=0; i<pcl_input_pc->points.size(); i++){
        const auto& p = pcl_input_pc->points[i];
        if(HEIGHT_THRESHOLD < p.z){
            pcl_output_pc->points.push_back(p);
        }
    }
    pc_publisher.publish(pcl_output_pc);
}

void IheightMap::executor(void)
{
    ros::Rate r((int)Hz);
    while(ros::ok()){
        r.sleep();
        ros::spinOnce();
    }
}