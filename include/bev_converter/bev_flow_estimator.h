#ifndef __BEV_FLOW_ESTIMATOR_H
#define __BEV_FLOW_ESTIMATOR_H

#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <opencv2/opencv.hpp>
/* #include <opencv_lib.hpp> */
#include <opencv2/superres/optical_flow.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include "bev_converter/bev_image_generator.h"

class BEVFlowEstimator
{
	public:
		BEVFlowEstimator(void);

		void executor(void);
        void formatter(void);
        void initializer(void);
		void grid_callback(const nav_msgs::OccupancyGridConstPtr&);
		void pre_grid_image_callback(const sensor_msgs::ImageConstPtr&);
		void odom_callback(const nav_msgs::OdometryConstPtr&);
        cv::Mat flow_estimator(cv::Mat&, cv::Mat&);

	private:
        XmlRpc::XmlRpcValue ROBOT_PARAM;

		bool first_flag = false;
		bool odom_callback_flag = false;
		bool grid_callback_flag = false;
        bool IS_SAVE_IMAGE;
        bool IS_DENSE;
        bool IS_LOCAL;

		constexpr static int Col = 0; //i↓  ...   ↑x
		constexpr static int Row = 1; //j→  ... y←o
		
		std::string PKG_PATH, FRAME_ID, CHILD_FRAME_ID;
        int GRID_NUM, SAVE_NUMBER, FLOW_IMAGE_SIZE, FLOW_WINiDOW_SIZE, MANUAL_CROP_SIZE, MAX_CORNERS, WIN_SIZE, MAX_COUNT, STEP_BORDER;
		int step;
        double RANGE, Hz, grid_size, dt, QUALITY_LEVEL, MIN_DISTANCE;
    	double run_length;
		double current_yaw, pre_yaw;

        ros::NodeHandle n;
        ros::NodeHandle nh;
        
		ros::Subscriber grid_subscriber;
		ros::Subscriber pre_grid_image_subscriber;
		ros::Subscriber odom_subscriber;
		// image_transport::Publisher flow_image_publisher;
		ros::Publisher flow_image_publisher;
		ros::Publisher bev_transformed_image_publisher;
		
        nav_msgs::OccupancyGrid bev_grid;

        cv::Mat input_grid_img;
        cv::Mat pre_input_grid_img;

		Eigen::Vector3d current_position;
		Eigen::Vector3d pre_position;
		
		// image_transport::ImageTransport it;
};

#endif// __BEV_FLOW_ESTIMATOR_H
