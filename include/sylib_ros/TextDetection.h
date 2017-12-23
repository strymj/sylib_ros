/**
* @file TextDetection.h
* @brief text detection library
* @author strymj
* @date 2017.05
*/

#ifndef TEXTDETECTION_H_
#define TEXTDETECTION_H_

#include <opencv2/core.hpp>
#include <tesseract/baseapi.h>


/** @brief sylib namespace */
namespace sy
{

	/** @brief text detection class */
	class TextDetection
	{
		public:

			TextDetection();
			TextDetection(std::string tessdata_path, std::string language);
			~TextDetection();

			std::string detection(cv::Mat& input_img, bool convert_gray_flag = true);
			std::string convertEncording(const std::string& str, const char* fromcode, const char* tocode);

		private:
			tesseract::TessBaseAPI* api;
			std::string result_text;


	};  // class TextSDetection

}  // namespace sy

#endif
