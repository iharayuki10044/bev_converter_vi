#ifndef __BEV_CONVERTER_H
#define __BEV_CONVERTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;

class IheightMap
{
    public:
        IheightMap(void);
        void executor(void);
        void initializer(void);
        void pc_callback(const sensor_msgs::PointCloud2ConstPtr&);

    private:
        bool pc_callback_flag;

        int Hz;
        double HEIGHT_THRESHOLD;

        ros::NodeHandle nh;
        ros::Subscriber pc_subscriber;
        ros::Publisher pc_publisher;

        CloudIPtr pcl_input_pc {new CloudI()};
        CloudIPtr pcl_output_pc {new CloudI()};
};

#endif