#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <QObject>
#include <types.h>
#include <ScanLines.h>
#include <RLE.h>

using namespace std;
using namespace cv;

class imageProcessor : public QObject
{
    Q_OBJECT
public:
    explicit imageProcessor(QObject *parent = 0);
    Point center;
public slots:
    // Image Output Functions
    void getBinary(Mat *in, int ymin, int ymax, int umin, int umax, int vmin, int vmax);
    void getSegmentedImage(Mat *in);
    void generateLookUpTable(int values[4][3][2]);
    bool writeLookUpTable();
    bool readLookUpTable();
    void preProcessIdx(ScanLines &linesRad, Mat *orig, Mat *idxImage);
    int getClassifier(Mat *buffer, int x, int y);
    void setCenter(Point center_);
private:
    hsv rgbtohsv(rgb in);
    int YUVLookUpTable[256*256*256];
};

#endif // IMAGEPROCESSOR_H
