#include "bev_converter/bev_flow_estimator.h"

BEVFlowEstimator::BEVFlowEstimator(void)
: nh("~")
{
    nh.param("RANGE", RANGE, {18.0});
    nh.param("GRID_NUM", GRID_NUM, {60});
    nh.param("Hz", Hz, {100.0});
    // nh.param("");
    nh.getParam("ROBOT_PARAM", ROBOT_PARAM);

    grid_subscriber = nh.subscribe("/bev/grid", 10, &BEVFlowEstimator::grid_callback, this);
    odom_subscriber = nh.subscribe("/estimated_pose/pose", 10, &BEVFlowEstimator::odom_callback, this);
    // bev_image_publisher = nh.advertise<>("/bev/image", 10);
    // bev_transformed_image_publisher = nh.advertise<>("/bev/transformed_image", 10);
}


void BEVFlowEstimator::execution(void)
{
	std::cout << "formatter" << std::endl;
    formatter();
    BEVImageGenerator bev_image_generator;
    bev_image_generator.formatter();

	ros::Rate r((int)Hz);
	while(ros::ok()){
		std::cout << "initializer" << std::endl;
        initializer();
        bev_image_generator.initializer();


		if(pc_callback_flag && odom_callback_flag){

			first_flag = true;
			grid_callback_flag = false;
			odom_callback_flag = false;
		}

		r.sleep();
		ros::spinOnce();
	}
}



void BEVFlowEstimator::formatter(void)
{
    dt = 1.0 / Hz;
    grid_size = RANGE / GRID_NUM;

}


void BEVFlowEstimator::initializer(void)
{
}



void BEVFlowEstimator::grid_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    bev_grid = *msg;
    
    double yaw, pitch, roll;
    geometry_msgs::Quaternion orientation = bev_grid.info.origin.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    mat.getEulerYPR(yaw, pitch, roll);
    double map_theta = yaw;

    input_grid_img = Mat::zeros(cv::Size(bev_grid.info.width,bev_grid.info.height), CV_8UC1);

    for(unsigned int col = 0; col < bev_grid.info.height; col++){
        for(unsigned int row = 0; row < bev_grid.info.width; row++){
            unsigned int i = row + (bev_grid.info.height - col - 1) * bev_grid.info.width;
            int intensity = 205;
            if(0 <= bev_grid.data[i] && brv_grid.data[i] <= 100){
                intensity = round((float)(100.0 - bev_grid.data[i]) * 2.55);
            }
            input_grid_img.at<unsigned char>(col, row) = intensity;
        }
    }

    grid_callback_flag = true;
}
