// #include{{{
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sensor_msgs/LaserScan.h>
/*}}}*/

bool scan_subscribed = false;
sensor_msgs::LaserScan::ConstPtr scan_ptr;
sensor_msgs::LaserScan scan_msg;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& msg)
{/*{{{*/
	scan_ptr = msg;
	scan_msg = *msg;
	scan_subscribed = true;
}/*}}}*/

void scanFix(sensor_msgs::LaserScan& scan, double inf_value)
{/*{{{*/
	// for (int i=0; i<scan.ranges.size(); ++i)
	// {
	// 	if (scan.range_max<scan.ranges[i])
	// 		scan.ranges[i] = inf_value;
	// }
	for (auto& el : scan.ranges)
	{
		if (scan.range_max<el)
			el = inf_value;
	}
}/*}}}*/

int main (int argc, char **argv)
{
	ros::init (argc, argv, "laser_fixer");
	ros::NodeHandle node_("~");
	ros::Rate looprate (30);

	std::string sub_topic, pub_topic;
	double inf_value;
	node_.param("sub_topic", sub_topic, std::string("scan"));
	node_.param("pub_topic", pub_topic, std::string("scan_fixed"));
	node_.param("inf_value", inf_value, 5.0);

	ros::Subscriber sub = node_.subscribe (sub_topic, 10, scanCallback);
	ros::Publisher pub = node_.advertise<sensor_msgs::LaserScan>(pub_topic, 10);

	while (ros::ok())
	{/*{{{*/
		if (scan_subscribed)
		{
			scanFix(scan_msg, inf_value);
			pub.publish(scan_msg);
		}

		scan_subscribed = false;
		ros::spinOnce();
		looprate.sleep();
	}/*}}}*/

	return 0;
}


