/**
* @file lidar2d.cpp
* @brief 2d lidar processing library
* @author strymj
* @date 2017.05
* @details not set
*/

#include <sylib_ros/Lidar2D.h>
#include <sylib_ros/Point.h>
using namespace std;
using namespace sy;


/**
 * @brief register lider scan 
 * @param (scan_input) input lidar scan data. (sensor_msgs::LaserScan::ConstPtr)
 * @return none
 */
void Lidar2D::setScanPtr(sensor_msgs::LaserScan::ConstPtr input_scan)
{/*{{{*/
	scan = input_scan;
}/*}}}*/


/**
 * @brief clustering 2d lider scan data
 * @param (tolerance) distance between two points in meters.
 * @param (cluster_min_n) minimum number of point in one cluster.
 * @return none
 */
void Lidar2D::clustering(double tolerance, int cluster_min_n)
{/*{{{*/
	Cluster cluster;
	clusterlist.clear();
	for(int i=0; i<scan->ranges.size(); ++i) {

		double angle = scan->angle_min + scan->angle_increment * i;
		sy::Point2D point(scan->ranges[i]*cos(angle), scan->ranges[i]*sin(angle));

		if(!isnan(scan->ranges[i]) && !isinf(scan->ranges[i]))
		{
			if (cluster.points.size() == 0 )
			{
				cluster.points.push_back(point);
				cluster.grav = point;
				cluster.N_bgn = i;
			}
			else if ( tolerance > point.distance(cluster.points.back()) )
			{
				cluster.points.push_back(point);
				cluster.grav.add(point);
			}
			else
			{
				if (cluster_min_n <= cluster.points.size())
				{
					cluster.grav.div(cluster.points.size());
					cluster.dist = cluster.grav.distance();
					cluster.width = cluster.points.front().distance( cluster.points.back() );
					cluster.N_end = i-1;
					cluster.angle = scan->angle_min + scan->angle_increment * (cluster.N_bgn+cluster.N_end)/2.0;
					clusterlist.push_back(cluster);
				}
				cluster.points.clear();
				--i;
			}
		}
	}
	clusterlist.push_back(cluster);
}/*}}}*/

