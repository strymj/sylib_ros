/**
 * @file image_proc.cpp
 * @brief image processing library
 * @author strymj
 * @date 2017.05
 */

#include <sylib_ros/ImageProc.h>
#include <sylib_ros/Labeling.h>
using namespace std;
using namespace cv;
using namespace sy;


/**
 * @brief extract spesific color
 * @param input_img   input image. (BGR)
 * @param output_img   output image. (GRAY)
 * @param low   hsv low threshold.
 * @param high   hsv high threshold.
 * @param blursize   Median blur kernel size. (It must be odd number.)
 * If the value is set to 0, this function don't apply median blur.
 */
void Image::colorExtract(Mat& input_img, Mat& output_img, HSV& low, HSV& high, int blursize)
{	/*{{{*/
	output_img = Mat::zeros(Size(input_img.cols, input_img.rows), CV_8UC1);
	Mat comp_img;
	cvtColor(input_img, comp_img, CV_BGR2HSV);
	if (low.hue <= high.hue) {
		for(int y=0; y<comp_img.rows; y++) {
			Vec3b *src_i = comp_img.ptr<Vec3b>(y);
			for(int x=0; x<comp_img.cols; x++) {
				if (low.hue <= src_i[x][0] && src_i[x][0] <= high.hue
					&& low.sat <= src_i[x][1] && src_i[x][1] <= high.sat
					&& low.val <= src_i[x][2] && src_i[x][2] <= high.val)
				{
					output_img.at<unsigned char>(y,x) = 255;
				}
			}
		}
	}
	else {
		for(int y=0; y<comp_img.rows; y++) {
			Vec3b *src_i = comp_img.ptr<Vec3b>(y);
			for(int x=0; x<comp_img.cols; x++) {
				if( (src_i[x][0] <= high.hue || low.hue <= src_i[x][0])
						&& low.sat <= src_i[x][1] && src_i[x][1] <= high.sat
						&& low.val <= src_i[x][2] && src_i[x][2] <= high.val)
				{
					output_img.at<unsigned char>(y,x) = 255;
				}
			}
		}
	}
	if (blursize != 0)
		medianBlur(output_img, output_img, blursize);
}/*}}}*/


/**
 * @brief labeling. 4 neighborhood connected region is regarded as one object.
 * @param binary_img   input image. (GRAY)
 * @param regiondata   regiondata vector for storeing result.
 * @param remove_pix   minimum value of region's size. (default: 200)
 */
void Image::labeling(Mat& binary_img, vector<Regiondata>& regiondata, unsigned int remove_pix)
{/*{{{*/
	regiondata.clear();
	Mat label(binary_img.size(), CV_16SC1);
	LabelingBS labeling;
	labeling.Exec(binary_img.data, (short *)label.data, binary_img.cols, binary_img.rows, true, remove_pix);	
	RegionInfoBS *ri;
	for( int i = 0; i < labeling.GetNumOfResultRegions(); i++)
	{
		Regiondata now;
		ri = labeling.GetResultRegionInfo(i);
		//Mat labelarea;
		//compare(label, i+1, labelarea, CV_CMP_EQ);
		// fighting!!! minEnclosingCircle(labelarea, data[i].c_center, data[i].radius);
		//Mat color(binary_img.size(), CV_8UC3, Scalar(255,255,255));
		//color.copyTo(label_img, labelarea);
		now.pixels = ri->GetNumOfPixels(); 
		Point2f center;
		ri->GetCenter(center.x, center.y); 
		now.center.x = center.x;
		now.center.y = center.y;
		ri->GetSize(now.size.x, now.size.y);
		ri->GetMin(now.min.x, now.min.y);
		ri->GetMax(now.max.x, now.max.y);
		now.color = ri->GetSourceValue();
		regiondata.push_back(now);
	}
}/*}}}*/
