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

#include <opencv2/opencv.hpp>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"


class BEVImageGenerator
{
	public:
		BEVImageGenerator(void);

        typedef struct Dynamics{
            float max_acceleration;
            float max_yawrate;
            float max_d_yawrate;
            float max_wheel_angular_velocyty;
            float wheel_radius;
            float tread;
        };
		typedef struct MyOdom{
			double x;
			double y;
			double z;
			double roll;
			double pitch;
			double yaw;
		};
        typedef struct OXY{
			Eigen::Vector2i o;
			Eigen::Vector2i x;
			Eigen::Vector2i y;
        }
		typedef struct UnitVectorOXY{
            OXY src;
            OXY dst;
		};

		void execution(void);
		void grid_callback(const nav_msgs::OccupancyGridConstPtr&);
		void odom_callback(const nav_msgs::OdometryConstPtr&);
        void formatter(void);
        void initializer(void);
        Eigen::Vector2i cell_motion_calculator(const int);
		void unit_vector_registrator(void);
        cv::Mat image_transformer(cv::Mat);
        cv::Mat image_cropper(cv::Mat);

	private:
        XmlRpc::XmlRpcValue ROBOT_PARAM;

		bool first_flag = false;
		bool grid_callback_flag = false;
		bool odom_callback_flag = false;
		bool tf_listen_flag = false;

		constexpr float Occupied = 1.0, Free = 0.0, Unknown = 0.5;
		constexpr int Col = 0; //i↓  ...   ↑x
		constexpr int Row = 1; //j→  ... y←o
        // constexpr int UV_O = 0, int UV_X = 1, int UV_Y = 2; // uv : unit vector
        constexpr std::map<std::string, int> UnitVector = {{"unit_vector_o", 1},
                                                           {"unit_vector_x", 2},
                                                           {"unit_vector_y", 3}};
        constexpr std::map<std::string, int> CropMode = {{"forward", 1},
                                                         {"rotate", 2}};

        int GRID_NUM, crop_size;
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

        Dynamics robot_param;
		MyOdom d_my_odom;
		UnitVectorOXY unit_vector;



};

#endif// __BEV_IMAGE_GENERATOR_H
