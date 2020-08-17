#ifndef __BEV_FLOW_ESTIMATOR_H
#define __BEV_FLOW_ESTIMATOR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <opencv2/opencv.hpp>
#include <opencv2/superres/optical_flow.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include "bev_image_generator"

class BEVFlowEstimator
{
	public:
		BEVFlowEstimator(void);

		void executor(void);
        void formatter(void);
        void initializer(void);
		void grid_callback(const nav_msgs::OccupancyGridConstPtr&);
        cv::Mat flow_estimator(cv::Mat&, cv::Mat&);

	private:
		bool first_flag = false;

		constexpr int Col = 0; //i↓  ...   ↑x
		constexpr int Row = 1; //j→  ... y←o

        int GRID_NUM, SAVE_NUMBER, FLOW_IMAGE_SIZE, crop_size;
        double RANGE, Hz, grid_size, dt;

        ros::NodeHandle n;
        ros::NodeHandle nh;
        
		ros::Subscriber grid_subscriber;
		ros::Subscriber odom_subscriber;
		ros::Publisher bev_image_publisher;
		ros::Publisher bev_transformed_image_publisher;
		
        nav_msgs::OccupancyGrid bev_grid;

        // Eigen::Vector3f zero_vector = Eigen::Vector3f::Zero();

        cv::Mat input_grid_img;
        cv::Mat pre_input_grid_img;

};

#endif// __BEV_FLOW_ESTIMATOR_H
