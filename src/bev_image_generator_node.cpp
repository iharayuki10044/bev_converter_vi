#include "bev_converter/bev_image_generator.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bev_image_generator");

	BEVImageGenerator bev_image_generator;
	bev_image_generator.execution();

	return 0;
}
