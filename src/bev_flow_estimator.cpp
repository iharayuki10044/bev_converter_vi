#include "bev_converter/bev_flow_estimator.h"

BEVFlowEstimator::BEVFlowEstimator(void)
: nh("~")
{
    nh.param("RANGE", RANGE, {10.0});
    nh.param("GRID_NUM", GRID_NUM, {50});
    nh.param("Hz", Hz, {100.0});
    nh.param("FLOW_IMAGE_SIZE", FLOW_IMAGE_SIZE, {50});
    nh.param("SAVE_NUMBER", SAVE_NUMBER, {1});
    nh.param("MANUAL_CROP_SIZE", MANUAL_CROP_SIZE, {5});
    nh.param("PKG_PATH", PKG_PATH, {"/home/amsl/ros_catkin_ws/src/bev_converter/bev_img"});
    nh.param("IS_SAVE_IMAGE", IS_SAVE_IMAGE, {false});
    nh.param("IS_DENSE", IS_SAVE_IMAGE, {false});
    nh.param("MAX_CORNERS", MAX_CORNERS, {20});
    nh.param("QUALITY_LEVEL", QUALITY_LEVEL, {0.05});
    nh.param("MIN_DISTANCE", MIN_DISTANCE, {5.0});
    nh.param("WIN_SIZE", WIN_SIZE, {3});
    nh.param("MAX_COUNT", MAX_COUNT, {30});
	nh.param("FRAME_ID", FRAME_ID, {"odom"});
	nh.param("CHILD_FRAME_ID", CHILD_FRAME_ID, {"base_footprint"});
    nh.param("STEP_BORDER", STEP_BORDER, {5});
    // nh.param("");
    nh.getParam("ROBOT_PARAM", ROBOT_PARAM);

    grid_subscriber = nh.subscribe("/bev/grid", 10, &BEVFlowEstimator::grid_callback, this);
	odom_subscriber = nh.subscribe("/odom", 10, &BEVFlowEstimator::odom_callback, this);
    /* pre_grid_image_subscriber = nh.subscribe("/bev/grid_image", 10, &BEVFlowEstimator::image_callback, this); */
	flow_image_publisher = nh.advertise<sensor_msgs::Image>("/bev/flow_image", 10);
    /* grid_subscriber = nh.subscribe("/dynamic_cloud_detector/occupancy_grid", 10, &BEVFlowEstimator::grid_callback, this); */
}


void BEVFlowEstimator::executor(void)
{
    formatter();
    BEVImageGenerator bev_image_generator(RANGE, GRID_NUM, MANUAL_CROP_SIZE, ROBOT_PARAM, FRAME_ID, CHILD_FRAME_ID);
    bev_image_generator.formatter();
    int i = 0;

	ros::Rate r(Hz);
	while(ros::ok()){
        bev_image_generator.initializer();

		if(odom_callback_flag && grid_callback_flag){
            initializer();
			cv::Mat cropped_current_grid_img = bev_image_generator.cropped_current_grid_img_generator(input_grid_img); // evry time newest

			if(step % STEP_BORDER == STEP_BORDER - 1){
        		cv::Mat cropped_transformed_grid_img = bev_image_generator.cropped_transformed_grid_img_generator(pre_input_grid_img, current_position, current_yaw, pre_position, pre_yaw);
				cv::Mat bev_flow;
				if(cropped_transformed_grid_img.size() == cropped_current_grid_img.size()){
					cropped_transformed_grid_img.convertTo(cropped_transformed_grid_img, CV_8U, 255);
					cropped_current_grid_img.convertTo(cropped_current_grid_img, CV_8U, 255);
					
					/* std::cout << "imshow" << std::endl; */
					/* cv::namedWindow("cropped_transformed_grid_img", CV_WINDOW_AUTOSIZE); */
					/* cv::imshow("cropped_transformed_grid_img", cropped_transformed_grid_img); */
					/* cv::waitKey(1); */
					/* cv::namedWindow("cropped_current_grid_img", CV_WINDOW_AUTOSIZE); */
					/* cv::imshow("cropped_current_grid_img", cropped_current_grid_img); */
					/* cv::waitKey(1); */

					bev_flow = flow_estimator(cropped_transformed_grid_img, cropped_current_grid_img);

					// cv::resize(bev_flow, bev_flow, cv::Size(FLOW_IMAGE_SIZE, FLOW_IMAGE_SIZE));
					// cv::rotate(bev_flow, bev_flow, cv::ROTATE_90_COUNTERCLOCKWISE);
					cv::flip(bev_flow, bev_flow, 0);
					bev_flow.convertTo(bev_flow, CV_8U, 255);

					/* std::cout << "imshow" << std::endl; */
					/* cv::namedWindow("bev_flow", CV_WINDOW_AUTOSIZE); */
					/* cv::imshow("bev_flow", bev_flow); */
					/* cv::waitKey(1); */
				}
				
				if(IS_SAVE_IMAGE){
					std::vector<int> params(2);
					// .png
					const std::string folder_name = PKG_PATH + "/data_" + std::to_string(SAVE_NUMBER);
					params[0] = CV_IMWRITE_PNG_COMPRESSION;
					params[1] = 9;

					struct stat statBuf;
					if(stat(folder_name.c_str(), &statBuf) == 0){
						std::cout << "exist dir" << std::endl;
					}else{
						std::cout << "mkdir" << std::endl;
						if(mkdir(folder_name.c_str(), 0755) != 0){
							std::cout << "mkdir error" << std::endl;
						}
					}
					/* cv::imwrite("/home/amsl/ros_catkin_ws/src/bev_converter/bev_img/data_" + std::to_string(SAVE_NUMBER) + "/" + "flow_" + std::to_string(i) + ".png", bev_flow, params); */
					cv::imwrite(folder_name + "/" + "flow_" + std::to_string(i) + ".png", bev_flow, params);
					/* std::cout << "SAVE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl; */
					i++;
				}

				std::cout << "pub img" << std::endl;
				cv::Mat flow_img;
				bev_flow.copyTo(flow_img);
				sensor_msgs::ImagePtr flow_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", flow_img).toImageMsg();
				flow_image_publisher.publish(flow_img_msg);
			}

			step++;
			// pre_input_grid_img = input_grid_img;
		}

		r.sleep();
		ros::spinOnce();
	}
}



void BEVFlowEstimator::formatter(void)
{
	/* std::cout << "formatter" << std::endl; */

    dt = 1.0 / Hz;
    grid_size = RANGE / GRID_NUM;
	step = 0;
}


void BEVFlowEstimator::initializer(void)
{
	/* std::cout << "initializer" << std::endl; */

}

void BEVFlowEstimator::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
    nav_msgs::Odometry odom;
	odom = *msg;
    
	current_position << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
    current_yaw = tf::getYaw(odom.pose.pose.orientation);

	if(step % STEP_BORDER == 0){
		pre_position = current_position;
		pre_yaw = current_yaw;
	}

	odom_callback_flag = true;
}


void BEVFlowEstimator::grid_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
	/* std::cout << "grid_callback" << std::endl; */

	nav_msgs::OccupancyGrid bev_grid = *msg;
    
    
    input_grid_img = cv::Mat::zeros(cv::Size(bev_grid.info.width,bev_grid.info.height), CV_8UC1);

    for(unsigned int col = 0; col < bev_grid.info.height; col++){
        for(unsigned int row = 0; row < bev_grid.info.width; row++){
            unsigned int i = row + (bev_grid.info.height - col - 1) * bev_grid.info.width;
            /* int intensity = 205; */
            /* if(0 <= bev_grid.data[i] && bev_grid.data[i] <= 100){ */
            /*     intensity = round((float)(100.0 - bev_grid.data[i]) * 2.55); */
            /* } */
            input_grid_img.at<unsigned char>(col, row) = bev_grid.data[i];
        }
    }

    if(step % STEP_BORDER == 0){
        pre_input_grid_img = input_grid_img;
    }

	grid_callback_flag = true;
}


cv::Mat BEVFlowEstimator::flow_estimator(cv::Mat &pre_img, cv::Mat &cur_img)
{
	std::cout << "flow_estimator" << std::endl;

    cv::Mat flow_bgr;
	int img_size = GRID_NUM - 2 * MANUAL_CROP_SIZE;
	cv::Mat flow_x = cv::Mat::zeros(img_size, img_size, CV_32F);
	cv::Mat flow_y = cv::Mat::zeros(img_size, img_size, CV_32F);

	if(IS_DENSE){
		cv::Ptr<cv::superres::DenseOpticalFlowExt> optical_flow = cv::superres::createOptFlow_DualTVL1();
		std::cout << "pre_img.size() = " << pre_img.size() << std::endl;
		std::cout << "cur_img.size() = " << cur_img.size() << std::endl;
		optical_flow->calc(pre_img, cur_img, flow_x, flow_y);
		cv::Mat magnitude, angle;
		cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);

		cv::Mat hsv_planes[3];
		hsv_planes[0] = angle;
		// cv::normalize(magnitude, magnitude, 0, 1, NORM_MINMAX);
		cv::normalize(magnitude, magnitude, 1.0, 0.0, cv::NORM_L1);
		hsv_planes[1] = magnitude;
		hsv_planes[2] = cv::Mat::ones(magnitude.size(), CV_32F);
		
		cv::Mat hsv;
		cv::merge(hsv_planes, 3, hsv);

		cv::cvtColor(hsv, flow_bgr, cv::COLOR_HSV2BGR);
	}else{
		std::vector<cv::Point2f> pre_corners;
		std::vector<cv::Point2f> cur_corners;
		cv::goodFeaturesToTrack(pre_img, pre_corners, MAX_CORNERS, QUALITY_LEVEL, MIN_DISTANCE);
		cv::goodFeaturesToTrack(cur_img, cur_corners, MAX_CORNERS, QUALITY_LEVEL, MIN_DISTANCE);
		std::cout << "pre_img.size() = " << pre_img.size() << std::endl;
		std::cout << "cur_img.size() = " << cur_img.size() << std::endl;
		std::cout << "pre_corners:" << pre_corners.size() << std::endl;
		std::cout << "cur_corners:" << cur_corners.size() << std::endl;
		if(pre_corners.size() > 0 && cur_corners.size() > 0){
			cv::cornerSubPix(pre_img, pre_corners, cv::Size(WIN_SIZE, WIN_SIZE), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, MAX_COUNT, QUALITY_LEVEL));
			cv::cornerSubPix(cur_img, cur_corners, cv::Size(WIN_SIZE, WIN_SIZE), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, MAX_COUNT, QUALITY_LEVEL));
			std::vector<uchar> features_found;
			std::vector<float> features_errors;
			cv::calcOpticalFlowPyrLK(pre_img, cur_img, pre_corners, cur_corners, features_found, features_errors);

			for(size_t i = 0; i < features_found.size(); i++){
				cv::Point flow_vector = cv::Point((int)(cur_corners[i].x - pre_corners[i].x), (int)(cur_corners[i].y - pre_corners[i].y));
				flow_x.at<int>((int)pre_corners[i].x, (int)pre_corners[i].y) = flow_vector.x;
				flow_y.at<int>((int)pre_corners[i].x, (int)pre_corners[i].y) = flow_vector.y;
			}
		}

		cv::Mat magnitude, angle;
		cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);

		cv::Mat hsv_planes[3];
		hsv_planes[0] = angle;
		cv::normalize(magnitude, magnitude, 1.0, 0.0, cv::NORM_MINMAX);
		/* cv::normalize(magnitude, magnitude, 1.0, 0.0, cv::NORM_L1); */
		hsv_planes[1] = magnitude;
		hsv_planes[2] = cv::Mat::ones(magnitude.size(), CV_32F);
		
		cv::Mat hsv;
		cv::merge(hsv_planes, 3, hsv);

		cv::cvtColor(hsv, flow_bgr, cv::COLOR_HSV2BGR);
	}

    return flow_bgr;
}




