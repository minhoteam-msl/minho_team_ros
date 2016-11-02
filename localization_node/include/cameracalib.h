#ifndef CAMERACALIB_H
#define CAMERACALIB_H

#include <opencv2/opencv.hpp>
#include "properties.h"

using namespace std;
using namespace cv;

class CameraCalib
{
public:
    CameraCalib(Properties *prop, Mat &imagePassed);
    ~CameraCalib();

    void calcLumiHistogram(Mat &image);
    void calcSatHistogram(Mat &image);
    Scalar averageRGB(Mat *image);
    float calcMean();
    Scalar averageUV(Mat *image);
    bool cameraCalibrate(Mat &mask);

    std::vector<int> histvalue;
    Mat image_roi_white,image_roi_black,image;
    void setRegions(Mat &image);
private:
    Properties *Proper;
    Rect region_of_interest;
    int x, y, w, h,x2, y2, w2, h2;
    float minError;
    bool firsttime;

};

#endif // CAMERACALIB_H