#ifndef TYPES
#define TYPES

#include <QString>
/*
######### DATA TYPES AND PATHS DEFINITION FOR IMAGEPROCESSOR.H/CPP #########
############################################################################
############################################################################
*/
// DIRECTORY PATHS
#define IMAGEFOLDERPATH "Images/"
#define FIELDSFOLDERPATH "Fields/"
#define CONFIGFOLDERPATH "/catkin_ws/src/minho_team_ros/localization_node/config/"
#define CCONFIGFOLDERPATH "/catkin_ws/src/minho_team_ros/control_node/config/"

// FILE NAMES
#define MIRRORFILENAME "mirror.cfg"
#define IMAGEFILENAME "image.cfg"
#define LUTFILENAME "lut.cfg"
#define MASKFILENAME "mask.png"
#define MAINFILENAME "main.cfg"
#define CONTROLFILENAME "control.cfg"
#define PIDFILENAME "pid.cfg"

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
typedef enum SortMethod{SORT_BY_DISTANCE=0, SORT_BY_SIZE} SortMethod;
typedef enum BlobType{OBS_BLOB=0, BALL_BLOB} BlobType;

/*
############################################################################
############################################################################
*/

/*
########## DATA TYPES AND PATHS DEFINITION FOR LOCALIZATION.H/CPP ##########
############################################################################
############################################################################
*/
#define IMG_SIZE 480

#define IMG_RAW 0x01
#define IMG_SEG 0x02
#define IMG_INT 0x04
#define IMG_WRL 0x08

#define HIST_SIZE 300.0
#define HIST_ANG 20

//########## ROBOT PHYSICAL PROPERTIES ##########
//###############################################
#define WHEELDIAMETER 0.1
#define DELTA_T 0.022  //20ms
#define DELTA_T_2 DELTA_T/2.0  //20ms
#define DEGTORAD (M_PI)/180.0
#define RADTODEG 180.0/(M_PI)
#define CPR 3036
#define DROB 0.235
#define COST 0.5
#define THETA_ 1.0471976
#define SINT 0.866
#define KWHEELS ((M_PI*WHEELDIAMETER)/(CPR*DELTA_T))
#define RROBOT 0.25
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

typedef struct field// Current field definitions
{
    unsigned int FIELD_POSITIONS;
    double FIELD_WIDTH,FIELD_LENGTH;
    double HALF_FIELD_WIDTH, HALF_FIELD_LENGTH;
    float TERM1, TERM2, TERM3;
    unsigned int MAX_LINE_POINTS;
    QString FIELD_NAME;
}field;

typedef struct nodo //World Map point of view
{
    double x;
    double y;
    double closestDistance;
    Vec2 gradient;
}nodo;

typedef struct gameField
{
    int TOTAL_LENGTH, TOTAL_WIDTH;
    int LENGTH, WIDTH;
    int GOAL_WIDTH, GOAL_LENGTH;
    int LINE_WIDTH;
    int CENTER_RADIUS;
    int SPOT_CENTER;
    int SPOTS;
    int AREA_LENGTH1, AREA_WIDTH1, AREA_LENGTH2, AREA_WIDTH2;
    int DISTANCE_PENALTY;
    int RADIUS_CORNER;
    int ROBOT_DIAMETER;
    int BALL_DIAMETER;
    int FACTOR;
    int GOALIE_LENGTH,GOALIE_WIDTH;
    int GOALIE_POST_WIDTH,GOALIE_BOARD_WIDTH;
} gameField;

typedef union fieldDimensions{
    struct gameField fieldDims;
    int dimensions[23];
} fieldDimensions;

/*
############################################################################
############################################################################
*/
#endif // TYPES
