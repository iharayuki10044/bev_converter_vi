#include "bev_converter/bev_image_generator.h"

BEVImageGenerator::BEVImageGenerator(double range, int grid_num)
: nh("~")
{
    RANGE = range;
    GRID_NUM = grid_num;

    // nh.param("");
    nh.getParam("ROBOT_PARAM", ROBOT_PARAM);

    odom_subscriber = nh.subscribe("/estimated_pose/pose", 10, &BEVImageGenerator::odom_callback, this);
}


cv::Mat BEVImageGenerator::cropped_current_grid_img_generator(cv::Mat& src_img)
{
    cv::Mat dst_img = image_cropper(src_img);

    return dst_img;
}


cv::Mat BEVImageGenerator::cropped_transformed_grid_img_generator(cv::Mat& src_img)
{
    cv::Mat transformed_grid_img, dst_img;

    if(odom_callback_flag){
        transformed_grid_img = image_transformer(src_img);
        dst_img = image_cropper(transformed_grid_img);
        odom_callback_flag = false;
    }

    return dst_img;
}




void BEVImageGenerator::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
	tf::Quaternion quat;
	geometry_msgs::Quaternion geometry_quat;
    nav_msgs::Odometry odom;
	double roll, pitch, yaw;

	odom = *msg;

    geometry_quat.x = odom.pose.pose.orientation.x;
    geometry_quat.y = odom.pose.pose.orientation.y;
    geometry_quat.z = odom.pose.pose.orientation.z;
    geometry_quat.w = odom.pose.pose.orientation.w;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	if(first_flag){
		pre_my_odom = now_my_odom;
	}

    now_my_odom.x = odom.pose.pose.position.x;
    now_my_odom.y = odom.pose.pose.position.y;
    now_my_odom.z = odom.pose.pose.position.z;
    now_my_odom.roll = roll;
    now_my_odom.pitch = pitch;
    now_my_odom.yaw = yaw;

	if(!first_flag){
		pre_my_odom = now_my_odom;
	}

	d_my_odom.x = now_my_odom.x - pre_my_odom.x;
	d_my_odom.y = now_my_odom.y - pre_my_odom.y;
	d_my_odom.z = now_my_odom.z - pre_my_odom.z;
	d_my_odom.roll = now_my_odom.roll - pre_my_odom.roll;
	d_my_odom.pitch = now_my_odom.pitch - pre_my_odom.pitch;
	d_my_odom.yaw = now_my_odom.yaw - pre_my_odom.yaw;

    odom_callback_flag = true;
}


void BEVImageGenerator::formatter(void)
{
    dt = 1.0 / Hz;
    grid_size = RANGE / GRID_NUM;

    robot_param.max_acceleration = ROBOT_PARAM["MAX_ACCELERATION"];
    robot_param.max_yawrate = ROBOT_PARAM["MAX_YAWRATE"];
    robot_param.max_d_yawrate = ROBOT_PARAM["MAX_D_YAWRATE"];
    robot_param.max_wheel_angular_velocyty = ROBOT_PARAM["MAX_WHEEL_ANGULAR_VELOCITY"];
    robot_param.wheel_radius = ROBOT_PARAM["WHEEL_RADIUS"];
    robot_param.tread = ROBOT_PARAM["TREAD"];

    int crop_size_forward = (int)(robot_param.wheel_radius * robot_param.max_wheel_angular_velocyty * dt / grid_size);
    int crop_size_rotate = (int)(0.5 * RANGE * tan(robot_param.max_yawrate * dt) / grid_size);
    if(crop_size_forward < crop_size_rotate){
        crop_size = crop_size_rotate;
        std::cout << "crop mode : rotate" << std::endl;
        std::cout << "crop_size : " << crop_size_rotate << std::endl;
    }else{
        crop_size = crop_size_forward;
        std::cout << "crop mode : forward" << std::endl;
        std::cout << "crop_size : " << crop_size_forward << std::endl;
    }

    unit_vector.src.o[Col] = GRID_NUM / 2;
    unit_vector.src.o[Row] = GRID_NUM / 2;
    unit_vector.src.x[Col] = GRID_NUM / 2 - 1;
    unit_vector.src.x[Row] = GRID_NUM / 2;
    unit_vector.src.y[Col] = GRID_NUM / 2;
    unit_vector.src.y[Row] = GRID_NUM / 2 - 1;
}


void BEVImageGenerator::initializer(void)
{
    unit_vector.dst = unit_vector.src;
}



Eigen::Vector2i BEVImageGenerator::cell_motion_calculator(std::string dim)
{
    switch(UnitVector[dim]){
        case 1: // unit_vector_o
            Eigen::Vector3d src_unit_vector << 0.0, 0.0, 1.0;
            break;
        case 2: // unit_vector_x
            Eigen::Vector3d src_unit_vector << 1.0, 0.0, 1.0;
            break;
        case 3: // unit_vector_y
            Eigen::Vector3d src_unit_vector << 0.0, 1.0, 1.0;
            break;
        default:
            Eigen::Vector3d src_unit_vector = Eigen::Vector3d::Zero();
            break;
    }

    Eigen::Vector3d homogenous_tf;
    Eigen::Vector2i cell_movement

    homogenous_tf(0, 0) = cos(d_my_odom.yaw);
    homogenous_tf(0, 1) = -sin(d_my_odom.yaw);
    homogenous_tf(1, 0) = sin(d_my_odom.yaw);
    homogenous_tf(1, 1) = cos(d_my_odom.yaw);
    homogenous_tf(0, 2) = d_my_odom.x / grid_size;
    homogenous_tf(1, 2) = d_my_odom.y / grid_size;
    homogenous_tf(2, 0) = 0.0;
    homogenous_tf(2, 1) = 0.0;
    homogenous_tf(2, 2) = 1.0;

    cell_movement = (int)(homogenous_tf * src_unit_vector);    

    return cell_movement;
}


void BEVImageGenerator::unit_vector_registrator(void)
{
    unit_vector.dst.o += cell_motion_calculator("unit_vector_o");
    unit_vector.dst.x += cell_motion_calculator("unit_vector_x");
    unit_vector.dst.y += cell_motion_calculator("unit_vector_y");
}


cv::Mat BEVImageGenerator::image_transformer(cv::Mat& src_img)
{
    unit_vector_registrator();
    const Point2f src_pt[] = {Point2f(unit_vector.src.o[Col], unit_vector.src.o[Row]),
                              Point2f(unit_vector.src.x[Col], unit_vector.src.x[Row]), 
                              Point2f(unit_vector.src.y[Col], unit_vector.src.y[Row])};
    const Point2f dst_pt[] = {Point2f(unit_vector.dst.o[Col], unit_vector.dst.o[Row]),
                              Point2f(unit_vector.dst.x[Col], unit_vector.dst.x[Row]), 
                              Point2f(unit_vector.dst.y[Col], unit_vector.dst.y[Row])};
    const cv::Mat affine_matrix = getAffineTransform(src_pt, dst_pt);
    cv::Mat dst_img;
    cv::warpAffine(src_img, dst_img, affine_matrix, src_img.size(), CV_INTER_LINEAR, cv::BORDER_TRANSPARENT);
    
    cv::line(src_img, src_pt[0], src_pt[1], Scalar(255,255,0), 2);
    cv::line(src_img, src_pt[1], src_pt[2], Scalar(255,255,0), 2);
    cv::line(src_img, src_pt[2], src_pt[0], Scalar(255,255,0), 2);
    cv::line(src_img, dst_pt[0], dst_pt[1], Scalar(255,0,255), 2);
    cv::line(src_img, dst_pt[1], dst_pt[2], Scalar(255,0,255), 2);
    cv::line(src_img, dst_pt[2], dst_pt[0], Scalar(255,0,255), 2);

    cv::namedWindow("src", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("dst", CV_WINDOW_AUTOSIZE);
    cv::imshow("src", src_img);
    cv::imshow("dst", dst_img);
    cv::waitKey(1000*(int)dt);

    return dst_img;
}


cv::Mat BEVImageGenerator::image_cropper(cv::Mat& src_img)
{
    cv::Rect roi(cv::Point(crop_size, crop_size), cv::Size(GRID_NUM - 2 * crop_size, GRID_NUM - 2 * crop_size));
    cv::Mat dst_img = src_img(roi);

    return dst_img;
}



