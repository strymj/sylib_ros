/**
 * @file ImageProc.h
 * @brief image processing library
 * @author strymj
 * @date 2017.05
 */

#ifndef IMAGEPROC_H_
#define IMAGEPROC_H_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

/** @brief sylib namespace */
namespace sy
{

	/** @brief image processing class */
	class Image 
	{
		public:

			/** @brief color structure for OpenCV's HSV parameter */
			struct HSV
			{/*{{{*/
				int hue;   /**< hue parameter. recommended 0-179 */
				int sat;   /**< satulation parameter. recommended 0-255 */
				int val;   /**< value parameter. recommended 0-255 */

				HSV(){};
				HSV(int h, int s, int v)
				{
					hue = h;
					sat = s;
					val = v;
				}

			};/*}}}*/

			/** @brief structure for labeling result */
			struct Regiondata
			{/*{{{*/
				int pixels;            /**< the number of pixels this region contains. */
				cv::Point2d center;    /**< rectangle center which contains this region. */
				cv::Point size;        /**< rectangle size. (width, height)*/
				cv::Point min;         /**< rectangle top left corner point. */
				cv::Point max;         /**< rectangle bottom right corner point. */
				unsigned char color;   /**< connected region's color. (gray scale) */
			};/*}}}*/

			static void colorExtract(cv::Mat& input_img, cv::Mat& output_img, HSV& low, HSV& high, int blursize);
			static void labeling(cv::Mat& input_img, std::vector<Regiondata>& regiondata, unsigned int remove_pix = 200);

		private:

	};   // class Image

}  // namespace sy

#endif
