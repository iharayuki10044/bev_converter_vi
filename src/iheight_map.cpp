#include "iheight_map/iheight_map.h"

IheightMap::IheightMap(void)
: nh("~")
{
    nh.param("Hz", Hz, {10});
    nh.param("MIN_HEIGHT", MIN_HEIGHT , {0.25});
    // 0.4 ball vanish

    pc_subscriber = nh.subscribe("/velodyne_points",10, &IheightMap::pc_callback, this);
    pc_publisher = nh.advertise<sensor_msgs::PointCloud2>("velodyne_obstacles", 1);
}

void IheightMap::pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 input_pc;
    input_pc = *msg;
    pcl::fromROSMsg(input_pc, *pcl_input_pc);

    int j=0;
    for(int i=0; i<pcl_input_pc->points.size(); i++){
        const auto& p = pcl_input_pc->points[i];
        if(MIN_HEIGHT < p.z){
            pcl_output_pc->points[j] = p;
            j++;
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