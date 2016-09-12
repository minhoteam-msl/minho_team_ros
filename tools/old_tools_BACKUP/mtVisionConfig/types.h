#ifndef TYPES_H
#define TYPES_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <flycapture/FlyCapture2.h>
#include <QThread>
#include <QCloseEvent>
#include <iostream>
#include <QDebug>
#include <QTcpSocket>
#include <QTcpServer>
#include <QHostAddress>
#include <QTimer>
#include <cstdlib>
#include <string>
#include <QTime>
#include <QNetworkInterface>

#define defaultPort 25321
#define defaultIP "127.0.0.1"

#define origFormat 0x01
#define segmFormat 0x02
#define binFormat 0x03
#define origGrabbing 0x04
#define segmGrabbing 0x05
#define worldGrabbing 0x06
#define histGrabbing 0x07
#define LMIN 0
#define LMAX 1

#define H 0
#define S 1
#define V 2

#define LUT_SIZE 256*256*256

typedef struct {
    double r;       // percent
    double g;       // percent
    double b;       // percent
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // percent
    double v;       // percent
} hsv;

#endif // TYPES_H

