#include "cameracalib.h"
#include "pid.h"

CameraCalib::CameraCalib(Properties *pro,Mat &imagePassed)
{
    //camera=cam;
    imagePassed.copyTo(image);
    x = y = w = h = x2 = y2 = w2 = h2 =0;
    minError=127*0.05;
    firsttime=true;
    Proper=pro;

}

///
/// \brief CameraCalib::calcLumiHistogram
///
void CameraCalib::calcLumiHistogram(Mat &image)
{
    Mat gray;

    cvtColor(image,gray,CV_RGB2GRAY);
    imshow("0 gray",gray);

    histvalue.clear();
    histvalue.resize(256);

    for(int i=0;i<gray.cols*image.rows;i++)
    {
            histvalue[(int)gray.ptr()[i]]++;
    }
}

///
/// \brief CameraCalib::calcSatHistogram
///
void CameraCalib::calcSatHistogram(Mat &image)
{
    Mat Hsv;

    cvtColor(image,Hsv,CV_RGB2HSV);
    imshow("0 sat",Hsv);

    histvalue.clear();
    histvalue.resize(256);

    for(int i=0;i<Hsv.cols*Hsv.rows;i++)
    {
        histvalue[(int)Hsv.ptr()[i]]++;
    }
}

///
/// \brief CameraCalib::averageRGB
/// \return
///
Scalar CameraCalib::averageRGB(Mat *image)
{
    int meanR=0,meanG=0,meanB=0,count=0;

    for(int i=0;i<image->cols;i++){
        for(int j=0;j<image->rows;j++){

            meanR = meanR + image->ptr()[(i*image->cols + j)*3];
            meanG = meanG + image->ptr()[(i*image->cols + j)*3 + 1];
            meanB = meanB + image->ptr()[(i*image->cols + j)*3 + 2];

            count++;
        }
    }
    meanR=meanR/count;
    meanG=meanG/count;
    meanB=meanB/count;


    Scalar s=Scalar(meanR,meanG,meanB);

    return s;
}

///
/// \brief CameraCalib::calcMean
/// \return
///
float CameraCalib::calcMean()
{
    float sum=0,temp=0;


    for(int i=0;i<256;i++)
    {
        sum=sum+i*histvalue[i];
        temp=temp+histvalue[i];
    }

    sum=sum/temp;
    return sum;
}

///
/// \brief CameraCalib::calcMeanUV
/// \return
///
Scalar CameraCalib::averageUV(Mat *image)
{
    int meanU=0,meanV=0,count=0;
    Mat temp;

    cvtColor(*image,temp,CV_RGB2YUV);

    for(int i=0;i<temp.cols;i++){
        for(int j=0;j<temp.rows;j++){

            meanU = meanU + temp.ptr()[(i*temp.cols + j)*3 + 1];
            meanV = meanV + temp.ptr()[(i*temp.cols + j)*3 + 2];

            count++;
        }
    }
    meanU=meanU/count;
    meanV=meanV/count;
    Scalar s= Scalar(meanU,meanV);

    return s;

}


void CameraCalib::setRegions(Mat &image){

    region_of_interest = Rect(x, y, w, h);
    image_roi_white = image(region_of_interest);//imagem
    region_of_interest = Rect(x2, y2, w2, h2);
    image_roi_black = image(region_of_interest);
}

bool CameraCalib::cameraCalibrate(Mat &mask)
{

    setRegions(mask);
    bool changed=false;
    //*******************************PID values start here******************************//
    static float WbKp=0.1;
    static float WbKi=0.0;

    static float WrKp=0.1;
    static float WrKi=0;

    static float GammaKp=5.2;
    static float GammaKi=0.0;

    static float SatKp=5.2;
    static float SatKi=0.0;

    static float BrigKp=5.2;
    static float BrigKi=0.0;

    static float GainKp=5.2;
    static float GainKi=0.0;

    static float ExpKp=5.2;
    static float ExpKi=0.0;

    static float msvError=0.0;

    //*******************************PID values end here******************************//

    float wblue=Proper->getWhite_Balance_valueB();
    float wred=Proper->getWhite_Balance_valueA();
    float gain=Proper->getGain();
    float gamma=Proper->getGamma();
    float sat=Proper->getSaturation();
    float brig=Proper->getBrigtness();
    float exp=Proper->getExposure();


    //Initialization of PID's
    PID wBluePID(WbKp,WbKi,0,Proper->getWhite_BalanceMin(),Proper->getWhite_BalanceMax());
    PID wRedPID(WrKp,WrKi,0,Proper->getWhite_BalanceMin(),Proper->getWhite_BalanceMax());
    PID GainPID(GainKp,GainKi,0,Proper->getGainMin(),Proper->getGainMax());
    PID GammaPID(GammaKp,GammaKi,0,Proper->getGammaMin(),Proper->getGainMax());
    PID SatPID(SatKp,SatKi,0,Proper->getSaturationMin(),Proper->getSaturationMax());
    PID BrigPID(BrigKp,BrigKi,0,Proper->getBrigtnessMin(),Proper->getBrigtnessMax());
    PID ExpPID(ExpKp,ExpKi,0,Proper->getExposureMin(),Proper->getExposureMax());


    //Initialization of some variables
    Scalar rgbMean=averageRGB(&image_roi_black);
    Scalar uvMean=averageUV(&image_roi_white);
    calcLumiHistogram(mask);
    int msv=calcMean();
    msvError=127-msv;

    if(firsttime){
        wBluePID.reset();
        wRedPID.reset();
        GainPID.reset();
        GammaPID.reset();
        SatPID.reset();
        BrigPID.reset();
        ExpPID.reset();
        firsttime=false;
    }


    if(msvError>minError || msvError<(-minError))
    {
        changed=true;
        if(gain==Proper->getGainMax() && exp==Proper->getExposureMax())
        {
            gamma=GammaPID.calc_pid(gamma,msvError);
            Proper->setGamma(gamma);
            Proper->setProps(GAM);
        }
        else
        {
            if(gain==Proper->getGainMax())
            {
                exp=ExpPID.calc_pid(exp,msvError);
                Proper->setExposure(exp);
                Proper->setProps(EXP);
            }
            else
            {
                gain=GainPID.calc_pid(gain,msvError);
                Proper->setGain(gain);
                Proper->setProps(GAI);
            }

        }
    }
    if(msv>101 && msv<152)
    {
        calcSatHistogram(mask);
        msvError=127-calcMean();
        if(msvError>minError || msvError<minError)
        {
            changed=true;
            sat=SatPID.calc_pid(sat,msvError);
            Proper->setSaturation(sat);
            Proper->setProps(SAT);
        }

        if(uvMean[0]>minError || uvMean[0]<(-minError) || uvMean[1]>minError || uvMean[1]<(-minError))
        {
            changed=true;
            wblue=wBluePID.calc_pid(wblue,minError-uvMean[0]);
            wred=wRedPID.calc_pid(wred,minError-uvMean[1]);
            Proper->setWhite_Balance(wred,wblue);
            Proper->setProps(WB);
        }
        float temp=-100;
        if(rgbMean[0]>minError || rgbMean[1]>minError || rgbMean[2]>minError)
        {
            changed=true;
            for(int i=0;i<3;i++){
                if(temp<rgbMean[i])temp=rgbMean[i];
            }
            brig=BrigPID.calc_pid(brig,temp);
            Proper->setBrigtness(brig);
            Proper->setProps(BRI);
        }
    }

    return changed;

}


