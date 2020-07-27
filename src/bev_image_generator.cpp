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
    odom_subscriber = nh.subscribe("/odom", 10, &BEVImageGenerator::odom_callback, this);
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

    	/* Eigen::Affine3d affine_transform; */
		/* try{ */
        /* 	tf::StampedTransform stamped_transform; */
        /* 	listener.lookupTransform("/odom", "/velodyne", ros::Time(0), stamped_transform); */
		/* 	Eigen::Translation<double, 3> t(stamped_transform.getOrigin().x(), stamped_transform.getOrigin().y(), stamped_transform.getOrigin().z()); */
		/* 	Eigen::Quaterniond q(stamped_transform.getRotation().w(), stamped_transform.getRotation().x(), stamped_transform.getRotation().y(), stamped_transform.getRotation().z()); */
		/* 	affine_transform = q * t; */
		/* 	tf_listen_flag = true; */
     	/* }    */
     	/* catch (tf::TransformException ex){ */
       	/* 	ROS_ERROR("%s",ex.what()); */
       	/* 	ros::Duration(1.0).sleep(); */
    	/* } */
		/* if(pc_callback_flag && odom_callback_flag && tf_listen_flag){ */
		/* 	#<{(| std::cout << "transform point cloud" << std::endl; |)}># */
        /* 	pcl::transformPointCloud(*pcl_filtered_pc, *pcl_transformed_pc, affine_transform); */
		/* } */

		if(pc_callback_flag && odom_callback_flag){

		}
		tf_listen_flag = false;
		grid_callback_flag = false;
		odom_callback_flag = false;

		r.sleep();
		ros::spinOnce();
	}
}


void BEVImageGenerator::pc_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    bev_grid = *msg;
    grid_callback_flag = true;
}


void BEVImageGenerator::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
    odom = *msg;
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
    cell_max_dynamics(robot);
}


void BEVImageGenerator::initializer(void)
{
}


CellDynamics BEVImageGenerator::cell_dynamics_calculator(Dynamics robot_dynamics)
{
    CellDynamics cell_dynamics;
    cell_dynamics.ix_vel = 

    return cell_dynamics;
}


nav_msgs::OccupancyGrid BEVImageGenerator::grid_transformer(nav_msgs::OccupancyGrid& grid)
{
}






