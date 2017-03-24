/* Class used for the implementation of scan lines. */

#ifndef SCANLINES_H_
#define SCANLINES_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include "Utils/types.h"

using namespace cv;
class ScanLines
{
public:

	cv::Point start; /** Starting point of the scan line. */
	cv::Point end; /** Ending point of the scan line. */
	unsigned stepX; /** Horizontal lines sparsity */
	unsigned stepY; /** Vertical lines sparsity */
	unsigned inRadius; /** Inner radius for radial scan lines.*/
	unsigned outRadius; /** Outer radius for radial scan lines.*/
	unsigned type; /** Type of the scanline (Radial, Circular, Horizontal or Vertical */
	cv::Point center; /** The center of the radial and circular scanlines */
	cv::Mat image; /** Reference image used to construct the Scanlines */
	std::vector <std::vector <int> > scanlines; /** Data container */

	ScanLines();
	/**
	 * Constructor for the creation of radial and circular scan lines.
	 */
    ScanLines (Mat &img,unsigned scanType, cv::Point center, unsigned nSensors,
			unsigned inRad, unsigned outRad, int radiusStep = 1);

    ScanLines (Mat &img,unsigned scanType, cv::Point center, unsigned nSensors,
				unsigned inRad, unsigned outRad, int nLevels, int step);

	/**
	 * Destructor.
	 */
	~ScanLines(){}

	int signum(int d);

	inline std::vector<int>& operator[](int idx) {return scanlines[idx];}

	inline const std::vector<int>& operator[](int idx) const {return scanlines[idx];}

	inline std::vector<int> getLine(int idx) {return scanlines[idx];}

	inline int getPoint(int i, int j) {return scanlines[i][j];}

	inline cv::Point getPointXYFromInteger(int idx)
	{
		cv::Point temp;

		temp.x = idx%image.cols;
		temp.y = idx/image.cols;

		return temp;
	}

	inline cv::Point getPointXY(int i, int j)
	{
		cv::Point temp;
		temp.x = scanlines[i][j]%image.cols;
		temp.y = scanlines[i][j]/image.cols;

		return temp;
	}

	unsigned getStep();

    void draw(cv::Mat &destination, cv::Scalar color);
    void drawLine(int pt1, int pt2, cv::Scalar color, cv::Mat &img);
    void drawCircle(int pt, int radius, cv::Scalar color, cv::Mat &img);
};

#endif
