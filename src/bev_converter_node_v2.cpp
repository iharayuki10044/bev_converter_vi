#include "bev_converter/bev_converter.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bev_converter");

	BEVConverter bev_converter;
	bev_converter.execution();

	return 0;
}
