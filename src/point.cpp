/**
* @file point.cpp
* @brief sylib point library
* @author strymj
* @date 2017.05
*/

#include <sylib_ros/Point.h>
using namespace sy;


/**
 * @brief constructer 
 */
Point2D::Point2D ()
{/*{{{*/
	x = 0.0;
	y = 0.0;
}/*}}}*/


/**
 * @brief constructer 
 * @param ix position x
 * @param iy position y
 */
Point2D::Point2D (double ix, double iy)
{/*{{{*/
	x = ix;
	y = iy;
}/*}}}*/


/**
 * @brief add another point
 * @param p point
 */
void Point2D::add(Point2D& p)
{/*{{{*/
	x += p.x;
	y += p.y;
}/*}}}*/


/**
 * @brief subtract another point
 * @param p point
 */
void Point2D::sub(Point2D& p)
{/*{{{*/
	x -= p.x;
	y -= p.y;
}/*}}}*/

/**
 * @brief scalar multiplication
 * @param d number
 */
void Point2D::mul(double d)
{/*{{{*/
	x *= d;
	y *= d;
}/*}}}*/

/**
 * @brief scalar division
 * @param d number
 */
void Point2D::div(double d)
{/*{{{*/
	x /= d;
	y /= d;
}/*}}}*/


/**
 * @brief calculate distance from origin
 * @param p point
 */
double Point2D::distance()
{/*{{{*/
	return sqrt( x*x + y*y );
}/*}}}*/


/**
 * @brief calculate distance from another point
 * @param p point
 */
double Point2D::distance(Point2D& p)
{/*{{{*/
	return sqrt( (x-p.x)*(x-p.x) + (y-p.y)*(y-p.y));
}/*}}}*/
