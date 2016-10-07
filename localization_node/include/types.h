#ifndef TYPES
#define TYPES


/*
######### DATA TYPES AND PATHS DEFINITION FOR IMAGEPROCESSOR.H/CPP #########
############################################################################
############################################################################   
*/
// DIRECTORY PATHS
#define imageFolderPath "Images/"
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

enum UAV_SCANLINES {UAV_HORIZONTAL = 0, UAV_VERTICAL, UAV_RADIAL, UAV_CIRCULAR};
enum UAV_COLORS {UAV_BLUE=0, UAV_YELLOW, UAV_ORANGE, UAV_GREEN, UAV_WHITE, UAV_BLACK, UAV_CYAN, UAV_MAGENTA, UAV_NOCOLORS };
enum UAV_COLORS_BIT {UAV_ORANGE_BIT = 32, UAV_BLACK_BIT = 4, UAV_GREEN_BIT = 16,
    UAV_WHITE_BIT = 8, UAV_BLUE_BIT = 128, UAV_YELLOW_BIT = 64, UAV_CYAN_BIT = 2,
    UAV_MAGENTA_BIT = 1, UAV_NOCOLORS_BIT = 0};

typedef enum LABEL_t {FIELD = 0, LINE, BALL, OBSTACLE} LABEL_t;
typedef enum COMPONENT_t { H = 0, S, V} COMPONENT_t;
typedef enum RANGE_t {MIN = 0, MAX} RANGE_t;

typedef struct labelConfiguration{
   int lb_calib[3][2] = {{0,180},{0,255},{0,255}};
} labelConfiguration;

typedef struct lutConfiguration{
   labelConfiguration lut_calib[4];   
} lutConfiguration;

/*
############################################################################
############################################################################   
*/

/*
########## DATA TYPES AND PATHS DEFINITION FOR LOCALIZATION.H/CPP ##########
############################################################################
############################################################################   
*/

#define IMG_RAW 0x01
#define IMG_SEG 0x02
#define IMG_WRL 0x03
#define IMG_MAP 0x04

/*
############################################################################
############################################################################   
*/
#endif // TYPES

