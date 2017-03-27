#ifndef BLOB_H_
#define BLOB_H_

#include "ScanLines.h"
#include "RLE.h"
#include "Common/Utils/types.h"
#include <iostream>

using namespace cv;
using namespace std;


class BlobInfo
{
public:

	Point2d center;
	vector<Point2d>points;
	double radius_id, dist_to_robot;

	BlobInfo(Point2d cen)
	{
		center=cen;
		points.push_back(cen);
		radius_id = 0.00;
		dist_to_robot = 0.00;
	}
};


class Blob
{

public:

	std::vector<BlobInfo> UMblobs;
	//RLE rle;

	Blob();

	inline Point getPointXYFromInteger(int idx, RLE &rle)
	{
		Point temp;

		temp.x = idx%rle.s.image.cols;
		temp.y = idx/rle.s.image.cols;

		return temp;
	}

	inline int getIntFromXY(Point pt, RLE &rle)
	{
		int temp;

		temp = pt.y * rle.s.image.cols + pt.x;

		return temp;
	}

	void createBlobs(vector<Point2d> &populationPoints, float threshold, int points_number, Point robotCenter, BlobType blobType);
	int findBlobReal(Point2d populationPoint, float thresh);
	void updateBlob(BlobInfo &blob, Point2d populationPoint, Point robotCenter);
	double distance(Point2d p1, Point2d p2);
	void draw(cv::Scalar color, cv::Mat *destination, double max_dist);
	void sort(SortMethod sortMethod);

private:
	void filterBlobs(int num, BlobType blobType);
	void seeNeighbor();
	void fuseBlobs(BlobInfo &b1, BlobInfo&b2);
	void updateFusedBlob(BlobInfo &blob);
	void relocBlobs();
	Point2d mapPointToRobot(double orientation, Point2d dist_lut);
};
#endif
