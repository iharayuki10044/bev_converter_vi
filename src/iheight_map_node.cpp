#include "iheight_map/iheight_map.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv,"/velodyne/obstacles");

	IheightMap iheight_map;
	iheight_map.executor();
	return 0;
}
