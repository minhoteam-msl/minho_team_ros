#ifndef TYPES
#define TYPES


/*
######### DATA TYPES AND PATHS DEFINITION FOR IMAGEPROCESSOR.H/CPP #########
############################################################################
############################################################################   
*/
// DIRECTORY PATHS
#define imgFolderPath "src/minho_team_ros/localization_node/config/Images/"
#define mainFile "src/minho_team_ros/localization_node/config/main.cfg"
#define configFolderPath "src/minho_team_ros/localization_node/config/"

// FILE NAMES
#define visParamsFileName "/visionparams.cfg"
#define lutFileName "/vision.cfg"
#define maskFileName "/mask.png"

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

