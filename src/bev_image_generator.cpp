#include "bev_converter/bev_image_generator.h"

BEVImageGenerator::BEVImageGenerator(void)
: nh("~")
{
    nh.param("WIDTH", WIDTH, {18.0});
    nh.param("HEIGHT", HEIGHT, {18.0});
    nh.param("GRID_NUM_X", GRID_NUM_X, {60});
    nh.param("GRID_NUM_Y", GRID_NUM_Y, {60});
    nh.param("Hz", Hz, {100.0});
    // nh.param("");
    nh.getParam("CATS_MOTION_PARAM", CATS_MOTION_PARAM);

    grid_subscriber = nh.subscribe("/bev/grid", 10, &BEVImageGenerator::grid_callback, this);
    odom_subscriber = nh.subscribe("/estimated_pose/pose", 10, &BEVImageGenerator::odom_callback, this);
    bev_image_publisher = nh.advertise<>("/bev/image", 10);
}


void BEVImageGenerator::execution(void)
{
	std::cout << "formatter" << std::endl;
    formatter();

	ros::Rate r((int)Hz);
	while(ros::ok()){
		std::cout << "initializer" << std::endl;
        initializer();


		if(pc_callback_flag && odom_callback_flag){

			first_flag = true;
			grid_callback_flag = false;
			odom_callback_flag = false;
		}

		r.sleep();
		ros::spinOnce();
	}
}


void BEVImageGenerator::grid_callback(const nav_msgs::OccupancyGridConstPtr &msg)
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


void BEVImageGenerator::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
	static MyOdom pre_my_odom;
	static MyOdom now_my_odom;
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

    grid_size_x = WIDTH / GRID_NUM_X;
    grid_size_y = HEIGHT / GRID_NUM_Y;
    grid_size_z = 0.5 * (grid_size_x + grid_size_y);

    robot_param.max_acceleration = CATS_MOTION_PARAM["MAX_ACCELERATION"];
    robot_param.max_yawrate = CATS_MOTION_PARAM["MAX_YAWRATE"];
    robot_param.max_d_yawrate = CATS_MOTION_PARAM["MAX_D_YAWRATE"];
    robot_param.max_wheel_angular_velocyty = CATS_MOTION_PARAM["MAX_WHEEL_ANGULAR_VELOCITY"];
    robot_param.wheel_radius = CATS_MOTION_PARAM["WHEEL_RADIUS"];
    robot_param.tread = CATS_MOTION_PARAM["TREAD"];
    max_motion.x = robot_param.wheel_radius * robot_param.max_wheel_angular_velocyty;
    max_motion.y = robot_param.wheel_radius * robot_param.max_wheel_angular_velocyty;
    max_motion.z = 0.0;
    max_motion.roll = 0.0;
    max_motion.pitch = 0.0;
    max_motion.yaw = robot_param.max_yawrate * dt;

    unit_vector.src.o[Col] = GRID_NUM_X / 2;
    unit_vector.src.o[Row] = GRID_NUM_Y / 2;
    unit_vector.src.x[Col] = GRID_NUM_X / 2 - 1;
    unit_vector.src.x[Row] = GRID_NUM_Y / 2;
    unit_vector.src.y[Col] = GRID_NUM_X / 2;
    unit_vector.src.y[Row] = GRID_NUM_Y / 2 - 1;
}


void BEVImageGenerator::initializer(void)
{
    unit_vector.dst = unit_vector.src;
}



Eigen::Vector2i BEVImageGenerator::cell_motion_calculator(int vector_)
{
    switch(vector_){
        case uv_o:
            Eigen::Vector3d src_unit_vector << 0.0, 0.0, 1.0;
        case uv_x:
            Eigen::Vector3d src_unit_vector << 1.0, 0.0, 1.0;
        case uv_y:
            Eigen::Vector3d src_unit_vector << 0.0, 1.0, 1.0;
        default:
            break;
    }

    Eigen::Vector3d homogenous_tf;
    // Eigen::Vector3d movement;
    Eigen::Vector2i cell_movement

    homogenous_tf(0, 0) = cos(d_my_odom.yaw);
    homogenous_tf(0, 1) = -sin(d_my_odom.yaw);
    homogenous_tf(1, 0) = sin(d_my_odom.yaw);
    homogenous_tf(1, 1) = cos(d_my_odom.yaw);
    homogenous_tf(0, 2) = d_my_odom.x / grid_size_x;
    homogenous_tf(1, 2) = d_my_odom.y / grid_size_y;
    homogenous_tf(2, 0) = 0.0;
    homogenous_tf(2, 1) = 0.0;
    homogenous_tf(2, 2) = 1.0;

    cell_movement = (int)(homogenous_tf * src_unit_vector);    

    return cell_movement;
}


UnitVectorOXY BEVImageGenerator::unit_vector_registrator(void)
{
    unit_vector.dst.o += cell_motion_calculator(uv_o);
    unit_vector.dst.x += cell_motion_calculator(uv_x);
    unit_vector.dst.y += cell_motion_calculator(uv_y);
}


void BEVImageGenerator::image_transformer(nav_msgs::OccupancyGrid &grid)
{
}






