#include "imageprocessor.h"

imageProcessor::imageProcessor(QObject *parent) : QObject(parent)
{
    center = Point(240,240);
}

void imageProcessor::getBinary(Mat *in, int ymin, int ymax, int umin, int umax, int vmin, int vmax)
{
    //Returns binary representation of a certain range
    Vec3b *pixel; // iterator to run through captured image
    rgb pix; hsv pix2;

    for(int i = 0; i < 480; ++i){
        pixel = in->ptr<Vec3b>(i);
        for (int j = 0; j<480; ++j){
            pix.r = pixel[j][2]; pix.g = pixel[j][1]; pix.b = pixel[j][0];
            pix2 = rgbtohsv(pix);

            if((pix2.h>=ymin)&&(pix2.h<=ymax) && (pix2.s>=umin)&&(pix2.s<=umax) && (pix2.v>=vmin)&&(pix2.v<=vmax))
            {
                pixel[j][2] = 255;
                pixel[j][1] = 255;
                pixel[j][0] = 255;
            }else {
                pixel[j][2] = 0;
                pixel[j][1] = 0;
                pixel[j][0] = 0;
            }
        }
    }
}

void imageProcessor::getSegmentedImage(Mat *in)
{
    Vec3b* pixel;
    long int index = 0;

    for(int i = 0; i < 480; ++i){
        pixel = in->ptr<Vec3b>(i);
        for (int j = 0; j< 480; ++j){
            index = (pixel[j][2]<<16) + (pixel[j][1]<<8) + (pixel[j][0]);
            if ( YUVLookUpTable[index] == UAV_GREEN_BIT)//se é campo
            {
                pixel[j][2] = 0;
                pixel[j][1] = 255;
                pixel[j][0] = 0;
            } else if ( YUVLookUpTable[index] == UAV_WHITE_BIT )//se é campo
            {
                pixel[j][2] = 255;
                pixel[j][1] = 255;
                pixel[j][0] = 255;
            }else if ( YUVLookUpTable[index] == UAV_ORANGE_BIT )//se é bola
            {
                pixel[j][2] = 255;
                pixel[j][1] = 0;
                pixel[j][0] = 0;
            }else if ( YUVLookUpTable[index] == UAV_BLACK_BIT)//se é obstaculo ou desconhecido
            {
                pixel[j][2] = 0;
                pixel[j][1] = 0;
                pixel[j][0] = 0;
            } else if ( YUVLookUpTable[index] == UAV_NOCOLORS_BIT)//se é obstaculo ou desconhecido
            {
                pixel[j][2] = 127;
                pixel[j][1] = 127;
                pixel[j][0] = 127;
            }
        }
    }

}

void imageProcessor::generateLookUpTable(int values[4][3][2])
{
    int y,u,v;
    rgb pix; hsv pix2;
    unsigned int index;
    for (int r=0; r<256; r++) // classify every RGB color into our LUT
        for (int g=0; g<256; g++)
            for (int b=0; b<256; b++)
            {
                pix.r = r; pix.g = g; pix.b = b;
                pix2 = rgbtohsv(pix);
                y = pix2.h;
                u = pix2.s;
                v = pix2.v;
                index = (r<<16)+(g<<8)+b;

                //-- initialize on update --
                YUVLookUpTable[index] = UAV_NOCOLORS_BIT;
                //-- Reference Colour range -UAV_GREEN_BIT-
                if (((y>=values[1][0][0]) && (y<=values[1][0][1])) && ((u>=values[1][1][0]) && (u<=values[1][1][1])) &&
                        ((v>=values[1][2][0]) && (v<=values[1][2][1]))){
                    YUVLookUpTable[index] = UAV_WHITE_BIT;
                }else if (((y>=values[0][0][0]) && (y<=values[0][0][1])) && ((u>=values[0][1][0]) && (u<=values[0][1][1])) &&
                          ((v>=values[0][2][0]) && (v<=values[0][2][1]))){
                    YUVLookUpTable[index] = UAV_GREEN_BIT;
                } else if (((y>=values[2][0][0]) && (y<=values[2][0][1])) && ((u>=values[2][1][0]) && (u<=values[2][1][1])) &&
                           ((v>=values[2][2][0]) && (v<=values[2][2][1]))){
                    YUVLookUpTable[index] = UAV_BLACK_BIT;
                }else if (((y>=values[3][0][0]) && (y<=values[3][0][1])) && ((u>=values[3][1][0]) && (u<=values[3][1][1])) &&
                          ((v>=values[3][2][0]) && (v<=values[3][2][1]))){
                    YUVLookUpTable[index] = UAV_ORANGE_BIT;
                }
            }
}

hsv imageProcessor::rgbtohsv(rgb in)
{
    hsv temp;
    int min = 0, max = 0, delta = 0;
    if(in.r<in.g)min=in.r; else min=in.g;
    if(in.b<min)min=in.b;

    if(in.r>in.g)max=in.r; else max=in.g;
    if(in.b>max)max=in.b;

    temp.v = max;                // v, 0..255
    delta = max - min;                      // 0..255, < v

    if(max != 0)
        temp.s = (int)(delta)*255/max;        // s, 0..255
    else {
        // r = g = b = 0        // s = 0, v is undefined
        temp.s = 0;
        temp.h = 0;
        return temp;
    }
    if(delta==0) temp.h = 0;
    else {
        if( in.r == max )
            temp.h = (in.g - in.b)*30/delta;        // between yellow & magenta
        else if( in.g == max )
            temp.h = 60 + (in.b - in.r)*30/delta;    // between cyan & yellow
        else
            temp.h = 120 + (in.r - in.g)*30/delta;    // between magenta & cyan

        while( temp.h < 0 ) temp.h += 180;
    }

    if(temp.h>160){
        temp.h = (int)(-0.11111*temp.h)+20;
    }

    if(temp.h==0) temp.h = 180;
    return temp;
}


bool imageProcessor::writeLookUpTable()
{
    QFile file("./Configs/vision.cfg");
    if(!file.open(QIODevice::WriteOnly)){
        qDebug() << "Error Writing vision.cfg";
        return false;
    }
    QTextStream in(&file);

    /* 255*255*255 values */
    QString toWrite = "";
    int counter = 0;
    for(int value = 0;value<LUT_SIZE;value++){
        if(value==LUT_SIZE-1){
            toWrite = QString::number(YUVLookUpTable[value]);
            counter++;
        } else {
            toWrite = QString::number(YUVLookUpTable[value])+",";
            counter+=2;
        }
        in << toWrite;
        toWrite.clear();
    }
    in << "\n";
    /* write distances */

    file.close();
    return true;
}

bool imageProcessor::readLookUpTable()
{
    QFile file("./Configs/vision.cfg");
    if(!file.open(QIODevice::ReadOnly)){
        qDebug() << "Error loading vision.cfg";
        return false;
    }

    QTextStream in(&file);

    QString line = in.readLine();
    QStringList values = line.split(",");

    /* 255*255*255 values */
    if(values.size()!=LUT_SIZE) {
        return false;
    }
    int val = UAV_NOCOLORS_BIT;
    for(int value = 0;value<LUT_SIZE;value++){
        val = values.at(value).toInt();
        YUVLookUpTable[value] = val;
    }

    file.close();
    return true;
}

void imageProcessor::preProcessIdx(ScanLines &linesRad, Mat *orig, Mat *idxImage)
{
    vector<int> s;
    Point temp;

    // Pre process radial sensors
    idxImage->setTo(0);
    int classifier = 0;
    for (unsigned k = 0 ; k < linesRad.scanlines.size() ; k++){
        s = linesRad.getLine(k);
        for(unsigned i = 0; i < s.size(); i++)
        {
            temp = linesRad.getPointXYFromInteger(s[i]);
            classifier = getClassifier(orig,temp.x,temp.y);
            idxImage->ptr()[s[i]] = classifier;
        }
    }
}

int imageProcessor::getClassifier(Mat *buffer, int x, int y)
{
    if(x<0 || x>=480 || y<0 || y>=480) return UAV_NOCOLORS_BIT;
    Vec3b *color = buffer->ptr<Vec3b>(y);
    long int index = (color[x][2]<<16) + (color[x][1]<<8) + (color[x][0]);
    return YUVLookUpTable[index];
}

void imageProcessor::setCenter(Point center_)
{
    center = center_;
}
