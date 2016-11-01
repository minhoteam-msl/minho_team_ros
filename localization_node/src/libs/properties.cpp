#include "properties.h"

Properties::Properties(Camera *cam)
{
    camera=cam;
    initProps();
}

///
/// \brief properties::setBrigtness
/// \param value
///
void Properties::setBrigtness(float value)
{
    props[0].absValue=value;
}
float Properties::getBrigtness()
{
    return props[0].absValue;
}

bool Properties::getBrigtnessState()
{
    return props[0].absControl;
}

void Properties::setBrigtnessState(bool state)
{
    props[0].absControl=state;
}

///
/// \brief Properties::setShutter
/// \param value
///
void Properties::setShutter(float value)
{
       props[1].absValue=value;
}
float Properties::getShuttertime()
{
    return props[1].absValue;
}
void Properties::setShutterState(bool state)
{
    props[1].absControl=state;
}
bool Properties::getShutterState()
{
    return props[1].absControl;
}

///
/// \brief ::setGain
/// \param value
///
void Properties::setGain(float value)
{
    props[2].absValue=value;
}
float Properties::getGain()
{
    return props[2].absValue;
}

bool Properties::getGainState()
{
    return props[2].absControl;
}

void Properties::setGainState(bool state)
{
    props[2].absControl=state;
}

///
/// \brief ::setExposure
/// \param value
///
void Properties::setExposure(float value)
{
    props[3].absValue=value;
}

float Properties::getExposure()
{
    return props[3].absValue;
}

bool Properties::getExposureState()
{
    return props[3].absControl;
}

void Properties::setExposureState(bool state)
{
    props[3].absControl=state;
}

///
/// \brief ::setGamma
/// \param value
///
void Properties::setGamma(float value)
{
    props[4].absValue=value;
}

float Properties::getGamma()
{
    return props[4].absValue;
}

bool Properties::getGammaState()
{
    return props[4].absControl;
}

void Properties::setGammaState(bool state)
{
    props[4].absControl=state;
}

///
/// \brief ::setHUE
/// \param value
///
void Properties::setHUE(float value)
{
    props[5].absValue=value;
}

float Properties::getHUE()
{
    return props[5].absValue;
}

bool Properties::getHUEState()
{
    return props[5].absControl;
}

void Properties::setHUEState(bool state)
{
    props[5].absControl=state;
}

///
/// \brief Properties::setSaturation
/// \param value
///
void Properties::setSaturation(float value)
{
    props[6].absValue=value;
}

float Properties::getSaturation()
{
    return props[6].absValue;
}

bool Properties::getSaturationState()
{
    return props[6].absControl;
}

void Properties::setSaturationState(bool state)
{
    props[6].absControl=state;
}

///
/// \brief Properties::setWhite_Balance
/// \param value
/// \param value2
///
void Properties::setWhite_Balance(float value, float value2)
{
    props[7].valueA=value;
    props[7].valueB=value2;
}

float Properties::getWhite_Balance_valueA()
{
    return props[7].valueA;
}

float Properties::getWhite_Balance_valueB()
{
    return props[7].valueB;
}

bool Properties::getWhite_BalanceState()
{
    return props[7].absControl;
}

void Properties::setWhite_BalanceState(bool state)
{
    props[7].absControl=state;
}


void Properties::setProps(int num)
{
    error=camera->SetProperty(&props[num]);
    if(error!=PGRERROR_OK)cout<<"Error Ocurred!! Impossible to set Properties!!"<<endl;
}

///
/// \brief Properties::initProps
///
void Properties::initProps()
{

    //Define the property to adjust.
    props[0].type = BRIGHTNESS;
    //Ensure the property is set up to use absolute value control.
    props[0].absControl = true;
    //Set the absolute value of brightness to 0.5%.
    props[0].absValue = 0.5;

    //Define the property to adjust.
    props[1].type = SHUTTER;
    //Ensure the property is on.
    props[1].onOff = true;
    //Ensure auto-adjust mode is off.
    props[1].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[1].absControl = true;
    //Set the absolute value of shutter to 20 ms.
    props[1].absValue = 20;

    props[2].type = GAIN;
    //Ensure auto-adjust mode is off.
    props[2].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[2].absControl = true;
    //Set the absolute value of gain to 10.5 dB.
    props[2].absValue = 10.5;

    props[3].type = AUTO_EXPOSURE;
    //Ensure the property is on.
    props[3].onOff = true;
    //Ensure auto-adjust mode is off.
    props[3].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[3].absControl = true;
    //Set the absolute value of auto exposure to -3.5 EV.
    props[3].absValue = -3.5;

    //Define the property to adjust.
    props[4].type = GAMMA;
    //Ensure the property is on.
    props[4].onOff = true;
    //Ensure the property is set up to use absolute value control.
    props[4].absControl = true;
    //Set the absolute value of gamma to 1.5
    props[4].absValue = 1.5;

    //Define the property to adjust.
    props[5].type = HUE;
    //Ensure the property is on.
    props[5].onOff = true;
    //Ensure the property is set up to use absolute value control.
    props[5].absControl = true;
    //Set the absolute value of hue to -30 deg.
    props[5].absValue = 15.293;

    //Define the property to adjust.
    props[6].type = SATURATION;
    //Ensure the property is on.
    props[6].onOff = true;
    //Ensure auto-adjust mode is off.
    props[6].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[6].absControl = true;
    //Set the absolute value of saturation to 200%.
    props[6].absValue=200;

    //Define the property to adjust.
    props[7].type = WHITE_BALANCE;
    //Ensure the property is on.
    props[7].onOff = true;
    //Ensure auto-adjust mode is off.
    props[7].autoManualMode = false;
    //Set the white balance red channel to 500.
    props[7].valueA = 500;
    //Set the white balance blue channel to 850.
    props[7].valueB = 850;
}

