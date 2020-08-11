#include "bev_converter/bev_flow_estimator.h"

BEVFlowEstimator::BEVFlowEstimator(void)
: nh("~")
{
    nh.param("RANGE", WIDTH, {18.0});
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



void BEVFlowEstimator::formatter(void)
{
    dt = 1.0 / Hz;
    grid_size = RANGE / GRID_NUM;

}


void BEVFlowEstimator::initializer(void)
{
}



