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

class BEVConverter
{
	public:
		BEVConverter(void);

		void execution(void);
		void pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
		void odom_callback(const nav_msgs::OdometryConstPtr&);
        pcl::PointCloud<PointI>::Ptr pc_downsampling(pcl::PointCloud<PointI>::Ptr);
        void formatter(void);
        void initializer(void);
        void converter(void);

	private:
		bool pc_callback_flag = false;
		bool odom_callback_flag = false;
		bool tf_listen_flag = false;

		constexpr static float Occupied = 1.0, Free = 0.0, Unknown = 0.5;

		double Hz;

        double RANGE;
        int GRID_NUM;
        float grid_size;
		std::string CHILD_FRAME_ID;

        ros::NodeHandle n;
        ros::NodeHandle nh;
        
		ros::Subscriber pc_subscriber;
		ros::Subscriber odom_subscriber;
		ros::Publisher bev_grid_publisher;
		
        nav_msgs::OccupancyGrid bev_grid;
        nav_msgs::Odometry odom;

		tf::TransformListener listener;
		tf::StampedTransform transform;

        CloudIPtr pcl_input_pc{new CloudI()};
        CloudIPtr pcl_filtered_pc {new CloudI()};
        CloudIPtr pcl_transformed_pc {new CloudI()};
};

#endif// __BEV_CONVERTER_H
