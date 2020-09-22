#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

class OdomPublisher
{
	public:
		OdomPublisher(void);

		void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr&);
		void initializer(void);

	private:
		std::string CMD_VEL_TOPIC, ODOM_TOPIC, FRAME_ID, CHILD_FRAME_ID;
		bool first_flag = false;
		int step;
		double Hz;

		ros::NodeHandle nh;
		ros::Subscriber cmd_vel_sub;
		ros::Publisher odom_pub;
		ros::Time current_time, last_time;
		nav_msgs::Odometry odom;

		tf::TransformListener listener;
		tf::StampedTransform transform;
};


OdomPublisher::OdomPublisher(void)
:nh("~")
{
	nh.param("CMD_VEL_TOPIC", CMD_VEL_TOPIC, {"/pedbot/control/cmd_vel"});
	nh.param("ODOM_TOPIC", ODOM_TOPIC, {"/odom"});
	nh.param("FRAME_ID", FRAME_ID, {"odom"});
	nh.param("CHILD_FRAME_ID", CHILD_FRAME_ID, {"base_footprint"});
	nh.param("Hz", Hz, {100.0});
	
	cmd_vel_sub = nh.subscribe(CMD_VEL_TOPIC, 10, &OdomPublisher::cmd_vel_callback, this);
	odom_pub = nh.advertise<nav_msgs::Odometry>(ODOM_TOPIC, 10);
}


void OdomPublisher::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
	geometry_msgs::Twist cmd_vel = *msg;

	if(!first_flag){
		initializer();
		first_flag = true;
	}

	try{
		listener.lookupTransform(FRAME_ID, CHILD_FRAME_ID, ros::Time(0), transform);
		current_time = ros::Time::now();
		
		odom.pose.pose.position.x = transform.getOrigin().x();
		odom.pose.pose.position.y = transform.getOrigin().y();
		odom.pose.pose.position.z = transform.getOrigin().z();
		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(transform.getRotation()));

	}   
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	} 

	odom.header.seq = step;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = FRAME_ID;
	odom.child_frame_id = CHILD_FRAME_ID;
	odom.twist.twist = cmd_vel;
	
	odom_pub.publish(odom);

	step++;
}


void OdomPublisher::initializer(void)
{
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	step = 0;
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 0.0;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "/bev_converter/pedsim/odom_publisher");
	
	OdomPublisher odom_publisher;
	ros::spin();

	return 0;
}
