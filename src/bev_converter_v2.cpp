#include "bev_converter/bev_converter_v2.h"

BEVConverter::BEVConverter(void)
: nh("~")
{
    nh.param("WIDTH", WIDTH, {18.0});
    nh.param("HEIGHT", HEIGHT, {18.0});
    nh.param("GRID_NUM_X", GRID_NUM_X, {60});
    nh.param("GRID_NUM_Y", GRID_NUM_Y, {60});
    nh.param("Hz", Hz, {100.0});
    // nh.param("");
    
    pc_subscriber = nh.subscribe("/velodyne_obstacles", 10, &BEVConverter::pc_callback, this);
    odom_subscriber = nh.subscribe("/odom", 10, &BEVConverter::odom_callback, this);
    bev_grid_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/bev/grid", 10);
}


void BEVConverter::execution(void)
{
	/* std::cout << "formatter" << std::endl; */

	ros::Rate r((int)Hz);
	while(ros::ok()){

		/* std::cout << "initializer" << std::endl; */
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
            if(!first_flag){
                formatter();
                first_flag = true;
            }

            bev_rotator();

			/* std::cout << "converter" << std::endl; */
            occupancy_gridder();

    		bev_grid.header.stamp = ros::Time::now();
    		bev_grid.info.map_load_time = ros::Time::now();
    		bev_grid_publisher.publish(bev_grid);
            
			//std::cout << bev_grid << std::endl;
		}
		tf_listen_flag = false;
		pc_callback_flag = false;
		odom_callback_flag = false;

		r.sleep();
		ros::spinOnce();
	}
}


void BEVConverter::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    sensor_msgs::PointCloud2 input_pc;

    input_pc = *msg;
	pcl::fromROSMsg(input_pc, *pcl_input_pc);
    pcl_filtered_pc = pc_downsampling(pcl_input_pc);
    pc_callback_flag = true;
}


pcl::PointCloud<PointI>::Ptr BEVConverter::pc_downsampling(pcl::PointCloud<PointI>::Ptr pcl_input_pc_)
{
    CloudIPtr pcl_filtered_pc_ {new CloudI()};
    CloudIPtr pcl_filtered_pc__ {new CloudI()};
    CloudIPtr pcl_filtered_pc___ {new CloudI()};
    pcl::VoxelGrid<pcl::PointXYZI> sor;

    sor.setInputCloud(pcl_input_pc_);
    sor.setLeafSize(grid_size_x, grid_size_y, grid_size_z);
    sor.filter(*pcl_filtered_pc_);

	/* pcl::PassThrough<PointI> pass; */
	/* pass.setInputCloud(pcl_filtered_pc_); */
	/* pass.setFilterFieldName ("x"); */
	/* pass.setFilterLimits(-0.5 * WIDTH, 0.5 * WIDTH); */
	/* pass.setFilterLimitsNegative (true); */
	/* pass.filter(*pcl_filtered_pc__); */
	/* pass.setInputCloud(pcl_filtered_pc__); */
	/* pass.setFilterFieldName ("y"); */
	/* pass.setFilterLimits(-0.5 * HEIGHT, 0.5 * HEIGHT); */
	/* pass.setFilterLimitsNegative (true); */
	/* pass.filter(*pcl_filtered_pc___); */

    // return pcl_filtered_pc___;
    return pcl_filtered_pc_;
}


void BEVConverter::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
    odom = *msg;
    odom_callback_flag = true;
}


void BEVConverter::formatter(void)
{
    bev_grid.header.seq = 0;
    bev_grid.header.frame_id = "velodyne";
    bev_grid.info.resolution = (float)(WIDTH / GRID_NUM_X);
    bev_grid.info.width = GRID_NUM_X;
    bev_grid.info.height = GRID_NUM_Y;
    bev_grid.info.origin = odom.pose.pose;
    bev_grid.info.origin.position.x -= 0.5 * WIDTH;
    bev_grid.info.origin.position.y -= 0.5 * HEIGHT;
    bev_grid.data.resize(GRID_NUM_X * GRID_NUM_Y);

    grid_size_x = WIDTH / GRID_NUM_X;
    grid_size_y = HEIGHT / GRID_NUM_Y;
    grid_size_z = 0.5 * (grid_size_x + grid_size_y);
}


void BEVConverter::initializer(void)
{
    for(auto& data : bev_grid.data){
        data = (int)Free;
    }
}

void BEVConverter::bev_rotator(void)
{
    double roll, pitch, yaw;
    tf::Quaternion quat;
    quaternionMsgToTF(odom.pose.pose, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    tf::Quaternion quat=tf::createQuaternionFromRPY(-roll, -pitch, -yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);

    bev_grid.info.origin.orientation.x += geometry_quat.x;
    bev_grid.info.origin.orientation.y += geometry_quat.y;
    bev_grid.info.origin.orientation.z += geometry_quat.z;
    bev_grid.info.origin.orientation.w += geometry_quat.w;
}


void BEVConverter::occupancy_gridder(void)
{
    /* for(auto& pt : pcl_transformed_pc->points){ */
    for(auto& pt : pcl_filtered_pc->points){
    // for(auto& pt : pcl_input_pc->points){
        int ix = floor((pt.x + 0.5 * WIDTH) / grid_size_x);
        int iy = floor((pt.y + 0.5 * HEIGHT) / grid_size_y);
        int index = ix + iy * (WIDTH / grid_size_x);
        if((0 <= ix && ix < GRID_NUM_X) && (0 <= iy && iy < GRID_NUM_Y)){
			/* std::cout << "[ix, iy] = [" << ix << ", " << iy << "]" << std::endl; */
            bev_grid.data[index] = (int)Occupied;
        }
    }
}









