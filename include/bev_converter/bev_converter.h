#ifndef __BEV_CONVERTER_H
#define __BEV_CONVERTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

// #include "Eigen/Core"
// #include "Eigen/Dense"
// #include "Eigen/LU"

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;

class BEVConverter
{
	public:
		BEVConverter(void);

		void execution(void);
		void pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
		void odom_callback(const geometry_msgs::PoseConstPtr&);
        pcl::PointCloud<PointI>::Ptr pc_downsampling(pcl::PointCloud<PointI>::Ptr);
        void formatting(void);
        void initialization(void);
        void converter(void);

	private:
		bool pc_callback_flag = false;
		bool odom_callback_flag = false;
		bool first_flag = false;

		constexpr static float Occupied = 1.0, Free = 0.0, Unknown = 0.5;

		double Hz;

        double WIDTH, HEIGHT; // x, y;
        int GRID_NUM_X, GRID_NUM_Y;
        float grid_size_x, grid_size_y, grid_size_z;

        ros::NodeHandle n;
        ros::NodeHandle nh;
        
		ros::Subscriber pc_subscriber;
		ros::Publisher bev_grid_publisher;
		
        nav_msgs::OccupancyGrid bev_grid;

        CloudIPtr pcl_input_pc{new CloudI()};
        CloudIPtr pcl_filtered_pc {new CloudI()};

        // Eigen::Vector3f zero_vector = Eigen::Vector3f::Zero();

};

#endif// __BEV_CONVERTER_H
