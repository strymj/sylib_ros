/**
* @file Lidar2D.h
* @brief 2d lidar processing library
* @author strymj
* @date 2017.05
* @details not set
*/

#ifndef LIDAR2D_H_
#define LIDAR2D_H_

#include <sylib_ros/Point.h>
#include <sensor_msgs/LaserScan.h>

/**
 * @brief sylib namespace
 */
namespace sy
{

	/**
	 * @brief 2d lidar processing class
	 */
	class Lidar2D 
	{
		public:
			/**
			 * @brief structure for storing cluster data
			 */
			struct Cluster
			{/*{{{*/
				std::vector<Point2D> points;
				sy::Point2D grav;
				double angle;
				double width;
				double dist;
				int N_bgn;
				int N_end;
			};/*}}}*/
			std::vector<Cluster> clusterlist;
			void setScanPtr (sensor_msgs::LaserScan::ConstPtr input_scan);
			void clustering (double tolerance, int cluster_min_n);

		private:
			sensor_msgs::LaserScan::ConstPtr scan;
	}; 

}  // namespace sy

#endif
