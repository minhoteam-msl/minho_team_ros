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

//########## ROBOT PHYSICAL PROPERTIES ########## 
//###############################################
#define wheelDiameter 0.1
#define deltaT 0.022  //20ms
#define deltaT_2 0.022/2.0  //20ms
#define degToRad (M_PI)/180.0
#define radToDeg 180.0/(M_PI)
#define CPR 3036
#define dRob 0.235
#define cost 0.5
#define theta_ 1.0471976
#define sint 0.866
#define KWheels ((M_PI*wheelDiameter)/(CPR*deltaT))
//###############################################
//###############################################
typedef struct localizationEstimate// Estimate of localization (vision, odometry, etc)
{
    float x;
    float y;
    float angle;
    float variance;
}localizationEstimate;

typedef struct Vec3{
   float x;
   float y;
   float z;
}Vec3;

typedef struct Vec2{
   float x;
   float y;
}Vec2;

typedef struct KalmanState{
    float x,y,thetaS, thetaC;
}KalmanState;

typedef struct MTKalmanFilter{
    Vec3 predictedState, predictedCovariance;
    Vec3 lastCovariance, covariance;
    Vec3 Q,R,K;
}MTKalmanFilter;

typedef struct Position{
    Vec2 pos;
    double heading;
    double error;
}Position;
/*
############################################################################
############################################################################   
*/
#endif // TYPES

