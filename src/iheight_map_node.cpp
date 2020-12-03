#include "iheight_map/iheight_map.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv,"iheight_map");

	IheightMap iheight_map;
	iheight_map.executor();
	return 0;
}
