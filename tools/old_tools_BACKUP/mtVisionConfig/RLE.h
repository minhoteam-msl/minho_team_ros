#ifndef RLE_H_
#define RLE_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include "ScanLines.h"

/** Structure for the definition of the run-length encoding information. */

struct RLEInfo
{
    int center, start, end, endAfter, startBefore;
    unsigned scIdx;
    unsigned lengthColorBefore;
    unsigned lengthColor;
    unsigned lengthColorAfter;

};

using namespace cv;
using namespace std;
class RLE
{

public:

	std::vector<RLEInfo> rlData;
	ScanLines s;
    RLE();
    RLE(ScanLines &s_, unsigned colorBefore, unsigned colorOfInterest, unsigned colorAfter,
            unsigned threshColorBefore, unsigned threshColorOfInterest, unsigned threshColorAfter,
            unsigned searchWindow);

	inline cv::Point getPointXYFromInteger(int idx)
	{
		cv::Point temp;
		temp.x = idx%s.image.cols;
		temp.y = idx/s.image.cols;

		return temp;
    }

    void draw(cv::Scalar colorBefore, cv::Scalar colorOfInterest, cv::Scalar colorAfter, cv::Mat &destination);
    void drawLine(int pt1, int pt2, cv::Scalar color, cv::Mat &img);
    void drawCircle(int pt, int radius, cv::Scalar color, cv::Mat &img);
    void drawInterestPoints(cv::Scalar color,cv::Mat &destination,UAV_COLORS_BIT idx);
    void pushData(std::vector<Point> &destination, Mat &img, UAV_COLORS_BIT idx);
};

#endif
