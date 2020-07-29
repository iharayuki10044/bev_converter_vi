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

	if(!first_flag){
		geometry_quat.x = odom.pose.pose.orientation.x;
		geometry_quat.y = odom.pose.pose.orientation.y;
		geometry_quat.z = odom.pose.pose.orientation.z;
		geometry_quat.w = odom.pose.pose.orientation.w;
		quaternionMsgToTF(geometry_quat, quat);
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

		now_my_odom.x = odom.pose.pose.position.x;
		now_my_odom.y = odom.pose.pose.position.y;
		now_my_odom.z = odom.pose.pose.position.z;
		now_my_odom.roll = roll;
		now_my_odom.pitch = pitch;
		now_my_odom.yaw = yaw;
		pre_my_odom = now_my_odom;
	}else{
		pre_my_odom = now_my_odom;

		geometry_quat.x = odom.pose.pose.orientation.x;
		geometry_quat.y = odom.pose.pose.orientation.y;
		geometry_quat.z = odom.pose.pose.orientation.z;
		geometry_quat.w = odom.pose.pose.orientation.w;
		quaternionMsgToTF(geometry_quat, quat);
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

		now_my_odom.x = odom.pose.pose.position.x;
		now_my_odom.y = odom.pose.pose.position.y;
		now_my_odom.z = odom.pose.pose.position.z;
		now_my_odom.roll = roll;
		now_my_odom.pitch = pitch;
		now_my_odom.yaw = yaw;
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
    grid_size_x = WIDTH / GRID_NUM_X;
    grid_size_y = HEIGHT / GRID_NUM_Y;
    grid_size_z = 0.5 * (grid_size_x + grid_size_y);

    robot.max_acceleration = CATS_MOTION_PARAM["MAX_ACCELERATION"];
    robot.max_yawrate = CATS_MOTION_PARAM["MAX_YAWRATE"];
    robot.max_d_yawrate = CATS_MOTION_PARAM["MAX_D_YAWRATE"];
    robot.max_wheel_angular_velocyty = CATS_MOTION_PARAM["MAX_WHEEL_ANGULAR_VELOCITY"];
    robot.wheel_radius = CATS_MOTION_PARAM["WHEEL_RADIUS"];
    robot.tread = CATS_MOTION_PARAM["TREAD"];
    cell_max_dynamics(robot_param);

	src_uv.o.col = GRID_NUM_X / 2;
	src_uv.o.row = GRID_NUM_Y / 2;
	src_uv.x.col = GRID_NUM_X / 2 - 1;
	src_uv.x.row = GRID_NUM_Y / 2;
	src_uv.y.col = GRID_NUM_X / 2;
	src_uv.y.row = GRID_NUM_Y / 2 - 1;
}


void BEVImageGenerator::initializer(void)
{
}


CellDynamics BEVImageGenerator::cell_dynamics_calculator(Dynamicsi &robot_dynamics)
{
    CellDynamics cell_dynamics;
    cell_dynamics.vel.col = 

    return cell_dynamics;
}


UnitVectorOXY BEVImageGenerator::unit_vector_registrator(void)
{
	if(!first_flag){
		dst_uv.o.col = GRID_NUM_X / 2;
		dst_uv.o.row = GRID_NUM_Y / 2;
		dst_uv.x.col = GRID_NUM_X / 2 - 1;
		dst_uv.x.row = GRID_NUM_Y / 2;
		dst_uv.y.col = GRID_NUM_X / 2;
		dst_uv.y.row = GRID_NUM_Y / 2 - 1;
	}else{
	}
	
}


void BEVImageGenerator::image_transformer(nav_msgs::OccupancyGrid &grid)
{
}






