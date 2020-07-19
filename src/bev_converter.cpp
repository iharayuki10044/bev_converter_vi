#include "bev_converter/bev_converter.h"

BEVConverter::BEVConverter(void)
: nh("~")
{
    nh.param("WIDTH", WIDTH, {18.0});
    nh.param("HEIGHT", HEIGHT, {18.0});
    nh.param("GRID_NUM_X", GRID_NUM_X, {60});
    nh.param("GRID_NUM_Y", GRID_NUM_Y, {60});
    // nh.param("");
    
    pc_subscriber = nh.subscribe("/velodyne_obstacles", 10, &BEVConverter::pc_callback, this);
    bev_grid_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/bev/grid", 10);
}


void BEVConverter::execution(void)
{
    formatting();

	ros::Rate r(Hz);
	while(ros::ok()){
        initialization();

		if(pc_callback_flag && odom_callback_flag){
            converter();
            pc_callback_flag = false;
            odom_callback_flag = false;
		}
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
    pcl::VoxelGrid<pcl::PointXYZI> sor;

    sor.setInputCloud(pcl_input_pc_);
    sor.setLeafSize(grid_size_x, grid_size_y, grid_size_z);
    sor.filter(*pcl_filtered_pc_);

    return pcl_filtered_pc_;
}


void BEVConverter::odom_callback(const geometry_msgs::PoseConstPtr &msg)
{
    odom = *msg;
    odom_callback_flag = true;
}


void BEVConverter::formatter(void)
{
    bev_grid.header.seq = 0;
    bev_grid.header.stamp = ros::Time::now();
    bev_grid.header.frame_id = "bev_map";
    bev_grid.info.map_load_time = ros::Time::now();
    bev_grid.info.resolution = (float)(WIDTH / GRID_NUM_X);
    bev_grid.info.width = WIDTH;
    bev_grid.info.height = HEIGHT;
    bev_grid.info.origin = odom;
    bev_grid.data.resize(WIDTH * HEIGHT);

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


void BEVConverter::converter(void)
{
    for(auto& pt : pcl_filtered_pc->points){
        int ix = (int)((pt.x + 0.5 * WIDTH) / grid_size_x);
        int iy = (int)((pt.y + 0.5 * HEIGHT) / grid_size_y);
        int index = ix + iy * (WIDTH / grid_size_x);
        if(index < WIDTH * HEIGHT){
            bev_grid.data[index] = (int)Occupied;
        }
    }

    bev_grid_publisher.publish(bev_grid);
}









