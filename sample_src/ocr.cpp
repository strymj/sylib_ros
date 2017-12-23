// #include{{{
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <sylib_ros/TextDetection.h>
/*}}}*/

using namespace std;

int main (int argc, char **argv)
{/*{{{*/
	ros::init (argc, argv, "ocr");
	ros::NodeHandle nh("~");

	string tessdata_path_, language_, image_path_;
	nh.param("tessdata_path", tessdata_path_, string("../"));
	nh.param("language", language_, string("eng"));
	nh.param("image_path", image_path_, string("../Pictures/sample.png"));

	cv::Mat image = cv::imread(image_path_);
	if (!image.data)
	{
		ROS_ERROR("Cannot load image at %s", image_path_.c_str());
		return -1;
	}

	sy::TextDetection td(tessdata_path_, language_);
	string result = td.detection(image, true);

	ROS_INFO("result : %s", result.c_str());

	return 0;
}

