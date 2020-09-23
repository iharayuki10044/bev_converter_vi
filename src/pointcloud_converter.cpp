#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>
#include <sensor_msgs/point_cloud_conversion.h>


class PointCloudConverter
{
	public:
		PointCloudConverter(void);
		
		void pc_callback(const sensor_msgs::PointCloudConstPtr&);

	private:
		std::string PEDSIM_PC_TOPIC;
		std::string PEDSIM_PC2_TOPIC;

        ros::NodeHandle nh;
		ros::Subscriber pc_subscriber;
		ros::Publisher pc2_publisher;
		
		sensor_msgs::PointCloud2 pointcloud2;
};


PointCloudConverter::PointCloudConverter(void)
: nh("~")
{
    nh.param("PEDSIM_PC_TOPIC", PEDSIM_PC_TOPIC, {"/pedsim_people_sensor/point_cloud_local"});
    nh.param("PEDSIM_PC2_TOPIC", PEDSIM_PC2_TOPIC, {"/pedsim_people_sensor/point_cloud2_local"});
	
    pc_subscriber = nh.subscribe(PEDSIM_PC_TOPIC, 10, &PointCloudConverter::pc_callback, this);
    pc2_publisher = nh.advertise<sensor_msgs::PointCloud2>(PEDSIM_PC2_TOPIC, 10);
}


void PointCloudConverter::pc_callback(const sensor_msgs::PointCloudConstPtr &pointcloud)
{
	if(sensor_msgs::convertPointCloudToPointCloud2(*pointcloud, pointcloud2)){
		pointcloud2.header = pointcloud->header;
		pointcloud2.height = pointcloud->points.size();
		pointcloud2.width = 1;
		pc2_publisher.publish(pointcloud2);
	}else{
		ROS_ERROR("[pointcloud_converter] ERROR");
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "/bev_converter/pointcloud_converter");
	
	PointCloudConverter pointcloud_converter;
	ros::spin();

	return 0;
}



