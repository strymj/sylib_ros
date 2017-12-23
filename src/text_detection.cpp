/**
* @file text_detection.cpp
* @brief text detection library
* @author strymj
* @date 2017.05
*/

#include <sylib_ros/TextDetection.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <iconv.h>

using namespace std;
using namespace cv;
using namespace sy;


/**
 * @brief constructer 
 */
TextDetection::TextDetection()
{/*{{{*/
		fprintf(stderr, "Please use Text(string, string) constructer.\n");
}/*}}}*/

/**
 * @brief constructer 
 * @param tessdata_path   tessdata directory path.
 * @param language   detection language. eng, jpn, ... and more.
 */
TextDetection::TextDetection(string tessdata_path, string language)
{/*{{{*/
	api = new tesseract::TessBaseAPI();
	if (api->Init(tessdata_path.c_str(), language.c_str())) {
		fprintf(stderr, "Could not initialize tesseract.\n");
	}
	result_text = "### no_data ###";
}/*}}}*/

/**
 * @brief destructer 
 */
TextDetection::~TextDetection()
{/*{{{*/
	api->End();
}/*}}}*/

/**
 * @brief text detection from image
 * @param input_img input image. (BGR or GRAY)
 * @param convert_gray_flag If this flag is set true, the input image will converted to gray image with otsu method.
 * @return detected string data.
 */
string TextDetection::detection(Mat& input_img, bool convert_gray_flag)
{/*{{{*/
	if (input_img.channels() != 1 && convert_gray_flag)
	{
		cv::cvtColor(input_img, input_img, CV_BGR2GRAY);
		cv::threshold(input_img, input_img, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
	}

	api->SetImage((uchar*)input_img.data, input_img.size().width,
			input_img.size().height, input_img.channels(), input_img.step1());
	char* result_char = api->GetUTF8Text();
	result_text = (string){result_char};

	return result_text;
}/*}}}*/

/**
 * @brief convert encording 
 * @param str input string.
 * @param fromcode input string's encording. ("iso-2022-jp", "UTF-8", "SHIFT-JIS", ...)
 * @param tocode output string's encording.
 * @return encorded string data.
 */
string TextDetection::convertEncording(const std::string& str, const char* fromcode, const char* tocode)
{/*{{{*/
    char *outstr, *instr;
    iconv_t icd;
    size_t instr_len  = std::strlen(str.c_str());
    size_t outstr_len = instr_len*2;
    
    if (instr_len <= 0) return "";
    
    // allocate memory
    instr  = new char[instr_len+1];
    outstr = new char[outstr_len+1];
    strcpy(instr, str.c_str());
    icd = iconv_open(tocode, fromcode);
    if (icd == (iconv_t)-1) {
        return "Failed to open iconv (" + std::string(fromcode) + " to " + std::string(tocode) + ")";
    }
    char *src_pos = instr, *dst_pos = outstr;
    if (iconv(icd, &src_pos, &instr_len, &dst_pos, &outstr_len) == -1) {
        // return error message
        std::string errstr;
        int err = errno;
        if (err == E2BIG) {
            errstr = "There is not sufficient room at *outbuf";
        } else if (err == EILSEQ) {
            errstr = "An invalid multibyte sequence has been encountered in the input";
        } else if (err = EINVAL) {
            errstr = "An incomplete multibyte sequence has been encountered in the input";
        }
        iconv_close(icd);
        return "Failed to convert string (" + errstr + ")";
    }
    *dst_pos = '\0';
    iconv_close(icd);
    
    std::string s(outstr);
    delete[] instr;
    delete[] outstr;
    
    return s;
}/*}}}*/

