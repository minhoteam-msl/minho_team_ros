#ifndef TYPES
#define TYPES

#include <QString>
/*
######### DATA TYPES AND PATHS DEFINITIONS #########
############################################################################
############################################################################
*/
// DIRECTORY PATHS
#define COMMON_PATH "/Common/"
#define IMAGES_PATH "Images/"
#define FIELDS_PATH "Fields/"
#define LOC_CFG_PATH "Configs/loc_cfg/"
#define CTRL_CFG_PATH "Configs/ctrl_cfg/"

// FILE NAMES
#define MIRRORFILENAME "mirror.cfg"
#define IMAGEFILENAME "image.cfg"
#define LUTFILENAME "lut.cfg"
#define MASKFILENAME "mask.png"
#define MAINFILENAME "fieldSelect.cfg"
#define CONTROLFILENAME "control.cfg"
#define PIDFILENAME "pid.cfg"
#define PROPERTYFILENAME "property.cfg"
#define WORLDFILENAME "worldconstruction.cfg"
#define KALMANFILENAME "kalman.cfg"

#define REFERENCE 2*2
#define DISTANCE 4*4
#define RELOC_COUNT 3
#define SEARCH_BALL_RADIUS 1.0
#define COUNT_BALL 60

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
########## TYPE DEFS ##########
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

/*
########## DATA TYPES ##########
############################################################################
############################################################################
*/

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
    double SCALE;
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


//############ WEIGHTS PARAMETERS ###############
//###############################################

#define REFERENCE 2*2
#define DISTANCE 4*4

#define LINE_LIMIT 4
//###############################################
//###############################################

/// \brief defines the type of roles available to choose
typedef enum Roles{rSTOP=0,rGOALKEEPER,rSUP_STRIKER,rSTRIKER} Roles;
/// \brief defines the game states known by the system
typedef enum GameState{sSTOPPED = 0,sPARKING,sPRE_OWN_KICKOFF,
sOWN_KICKOFF,sPRE_THEIR_KICKOFF,sTHEIR_KICKOFF,sPRE_OWN_FREEKICK,
sOWN_FREEKICK,sPRE_THEIR_FREEKICK, sTHEIR_FREEKICK,sGAME_OWN_BALL,
sGAME_THEIR_BALL} GameState;
/// \brief defines the actions wich model the robots control approach
typedef enum Actions{aSTOP = 0, aAPPROACHPOSITION, aFASTMOVE, aAPPROACHBALL,
aENGAGEBALL, aRECEIVEBALL, aPASSBALL, aKICKBALL, aDRIBBLEBALL, aHOLDBALL}Actions;

// aSTOP -> Remain stopped
// aAPPROACHPOSITION -> Approach a determined position with precision, with slow movement,
//                      suitable for PRE states
// aFASTMOVE -> Go to a position as fast and stable as possible. Suitable to intercept balls
//              , cover a player or do attack/defensive plays.
// aAPPROACHBALL -> Go to a position near game ball, considering it an obstacle beacause it
//                  should not be touched. 
// aENGAGEBALL -> Engage ball, with the objective of grabbing it
// aRECEIVEBALL -> Receive ball, but without engaging it (wait for the ball to come)
// aPASSBALL -> Pass the ball to a determined x-y position
// aKICKBALL -> Kick the ball to a determined position
// aDRIBBLEBALL -> Dribble the ball to a determined position
// aHIDEBALL -> Hide and hold ball, avoiding it being retaked by other players

/*
############################################################################
############################################################################
*/
#endif // TYPES
