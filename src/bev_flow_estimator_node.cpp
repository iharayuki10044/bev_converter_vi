#include "bev_converter/bev_flow_estimator.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bev_flow_estimator");

	BEVFlowEstimator bev_flow_estimator;
	bev_flow_estimator.execution();

	return 0;
}
