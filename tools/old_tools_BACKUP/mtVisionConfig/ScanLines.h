/* Class used for the implementation of scan lines. */

#ifndef SCANLINES_H_
#define SCANLINES_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <QDebug>

enum UAV_SCANLINES {UAV_HORIZONTAL = 0, UAV_VERTICAL, UAV_RADIAL, UAV_CIRCULAR};
enum UAV_COLORS {UAV_BLUE=0, UAV_YELLOW, UAV_ORANGE, UAV_GREEN, UAV_WHITE, UAV_BLACK, UAV_CYAN, UAV_MAGENTA, UAV_NOCOLORS };
enum UAV_COLORS_BIT {UAV_ORANGE_BIT = 32, UAV_BLACK_BIT = 4, UAV_GREEN_BIT = 16,
    UAV_WHITE_BIT = 8, UAV_BLUE_BIT = 128, UAV_YELLOW_BIT = 64, UAV_CYAN_BIT = 2,
    UAV_MAGENTA_BIT = 1, UAV_NOCOLORS_BIT = 0};

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

    ScanLines (Mat &img, cv::Point center_, cv::Point end_);

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
