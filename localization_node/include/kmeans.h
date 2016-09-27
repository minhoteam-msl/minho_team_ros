#ifndef KMEANS_H
#define KMEANS_H

//#include "Localization_Module/minhotypes.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

struct cluster{
    Point center;
    vector<Point>members;
    int totalDisparity;
};

class kMeans
{
public:
    kMeans(vector<Point>& population, int kClusters, int maxClusterDisparity);
    vector<Point> getClusters();
private:
    vector<Point> pop;
    unsigned int K;
    unsigned int maxCDisp;
    unsigned int maxIterations;
    int nearestCluster(Point point, vector<cluster> &clusters);
    int squaredDist(Point p1, Point p2);
};

#endif // KMEANS_H
