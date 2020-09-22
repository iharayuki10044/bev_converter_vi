#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
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

		ros::NodeHandle nh;
		ros::Subscriber cmd_vel_sub;
		ros::Publisher odom_pub;
		nav_msgs::Odometry odom;
};


OdomPublisher::OdomPublisher(void)
:nh("~")
{
	nh.param("CMD_VEL_TOPIC", CMD_VEL_TOPIC, {"/pedbot/control/cmd_vel"});
	nh.param("ODOM_TOPIC", ODOM_TOPIC, {"/odom"});
	nh.param("FRAME_ID", FRAME_ID, {"odom"});
	nh.param("CHILD_FRAME_ID", CHILD_FRAME_ID, {"base_footprint"});
	
	cmd_vel_sub = nh.subscribe(CMD_VEL_TOPIC, 10, &OdomPublisher::cmd_vel_callback, this);
	odom_pub = nh.advertise<nav_msgs::Odometry>(ODOM_TOPIC, 10);
}


void OdomPublisher::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
	ros::Time current_time = ros::Time::now();

	if(!first_flag){
		initializer();
		first_flag = true;
	}

	geometry_msgs::Twist cmd_vel = *msg;

	odom.header.seq = step;
	odom.header.stamp = current_time;
	odom.header.frame_id = FRAME_ID;
	odom.child_frame_id = CHILD_FRAME_ID;
	odom.twist.twist = cmd_vel;
	
	odom.pose.pose.position.x += odom.twist.twist.linear.x * cos(odom.twist.twist.angular.z);
	odom.pose.pose.position.y += odom.twist.twist.linear.x * sin(odom.twist.twist.angular.z);
	
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom.twist.twist.angular.z);
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = FRAME_ID;
	odom_trans.child_frame_id = CHILD_FRAME_ID;
	odom_trans.transform.translation.x = odom.pose.pose.position.x;
	odom_trans.transform.translation.y = odom.pose.pose.position.y;
	odom_trans.transform.translation.z = odom.pose.pose.position.z;
	odom_trans.transform.rotation = odom_quat;
	
	//send the transform
	tf::TransformBroadcaster odom_broadcaster;
	odom_broadcaster.sendTransform(odom_trans);
	
	odom.pose.pose.orientation = odom_quat;
	
	odom_pub.publish(odom);

	step++;
}


void OdomPublisher::initializer(void)
{
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
