#ifndef TYPES
#define TYPES


/*
######### DATA TYPES AND PATHS DEFINITION FOR IMAGEPROCESSOR.H/CPP #########
############################################################################
############################################################################   
*/
// DIRECTORY PATHS
#define imgFolderPath "Images/"
#define fieldsFolderPath "Fields/"
#define configFolderPath "/catkin_ws/src/minho_team_ros/localization_node/config/"

// FILE NAMES
#define mirrorFileName "mirror.cfg"
#define imageFileName "image.cfg"
#define lutFileName "lut.cfg"
#define maskFileName "mask.png"
#define mainFileName "main.cfg"

// LUT DEFINES
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

/*
############################################################################
############################################################################   
*/


#endif // TYPES

