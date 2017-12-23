/**
* @file Point.h
* @brief point library
* @author strymj
* @date 2017.05
* @details not set
*/

#ifndef POINT_H_
#define POINT_H_

#include <cmath>

/**
 * @brief sylib namespace
 */
namespace sy 
{
	/**
	 * @brief point class
	 */
	class Point2D
	{/*{{{*/
		public:
			double x;
			double y;

			Point2D ();
			Point2D (double, double);

			void add(Point2D& p);
			void sub(Point2D& p);
			void mul(double d);
			void div(double d);
			double distance();
			double distance(Point2D& p);
			static double distance(Point2D& p1, Point2D& p2);
	};/*}}}*/

}   // namespace sy

#endif
