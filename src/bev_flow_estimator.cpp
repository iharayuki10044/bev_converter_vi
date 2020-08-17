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


void BEVFlowEstimator::executor(void)
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

		if(grid_callback_flag){
            cropped_current_grid_img = bev_image_generator.cropped_current_grid_img_generator(input_grid_img);
            cropped_transformed_grid_img = bev_image_generator.cropped_transformed_grid_img_generator(pre_input_grid_img);
            cv::Mat bev_flow = flow_estimator(cropped_transformed_grid_img, cropped_current_grid_img);

			first_flag = true;
			grid_callback_flag = false;
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

    if(first_flag){
        pre_input_grid_img = input_grid_img;
    }
    
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

    if(!first_flag){
        pre_input_grid_img = input_grid_img;
    }

    grid_callback_flag = true;
}


cv::Mat BEVFlowEstimator::flow_estimator(cv::Mat& pre_img, cv::Mat& cur_img)
{
    Ptr<DenseOpticalFlowExt> optical_flow = superres::createOptFlow_DualTVL1();
    cv::Mat flow_x, flow_y;
    optical_flow->calc(pre_img, cur_img, flow_x, flow_y);

    cv::Mat magnitude, angle;
    cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);
    cv::Mat hsv_planes[3];
    hsv_planes[0] = angle;
    cv::normalize(magnitude, magnitude, 0, 1, NORM_MINMAX);
    hsv_planes[1] = magnitude;
    hsv_planes[2] = cv::Mat::ones(magnitude.size(), CV_32F);
    
    cv::Mat hsv;
    cv::merge(hsv_planes, 3, hsv);

    cv::Mat flow_bgr;
    cvCvtColor(hsv, flow_bgr, cv::COLOR_HSV2BGR);

    return flow_bgr;
}




