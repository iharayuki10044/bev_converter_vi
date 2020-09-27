#ifndef __BEV_IMAGE_GENERATOR_H
#define __BEV_IMAGE_GENERATOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"


class BEVImageGenerator
{
	public:
		BEVImageGenerator(double, int, int, XmlRpc::XmlRpcValue, std::string, std::string);

		cv::Mat cropped_current_grid_img_generator(cv::Mat);
		cv::Mat cropped_transformed_grid_img_generator(cv::Mat, Eigen::Vector3d, double, Eigen::Vector3d, double);
		void odom_callback(const nav_msgs::OdometryConstPtr&);
        void exector(void);
        void formatter(void);
        void initializer(void);
        cv::Mat image_transformer(cv::Mat);
        cv::Mat image_cropper(cv::Mat);

	private:
        XmlRpc::XmlRpcValue ROBOT_PARAM;

		bool odom_callback_flag;
		bool first_flag;

        int GRID_NUM, MANUAL_CROP_SIZE;
		std::string FRAME_ID, CHILD_FRAME_ID;
        int crop_size;
        double RANGE, elapsed_time;
        double Hz, dt, grid_size;

        ros::NodeHandle n;
        ros::NodeHandle nh;
        
		ros::Subscriber grid_subscriber;
		ros::Subscriber odom_subscriber;
		ros::Publisher bev_image_publisher;
		ros::Publisher bev_transformed_image_publisher;
		
        // Eigen::Vector3f zero_vector = Eigen::Vector3f::Zero();
    	Eigen::Affine3d affine_transform;

		pcl::PointXYZ pt0, pt1, pt2;
		pcl::PointCloud<pcl::PointXYZ> src_euqlid_3pts;
		pcl::PointCloud<pcl::PointXYZ> dst_euqlid_3pts;
};

#endif// __BEV_IMAGE_GENERATOR_H
