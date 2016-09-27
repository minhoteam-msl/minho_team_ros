#ifndef MINHOTYPES
#define MINHOTYPES

#define USING_ROS //UNCOMMENT if ROS is not installed in your system
#define CATKIN_COMPILE //UNCOMMENT if running exec with Qt
//ROS includes for ROS console
#ifdef USING_ROS
    #include "ros/ros.h"
#endif

#ifdef CATKIN_COMPILE
    //Path definition for Catkin Compilation
    #define imgPath "src/minho_team/vision_node/config/Images/"
    #define mainFile "src/minho_team/vision_node/config/main.cfg"
    #define officialMap "src/minho_team/vision_node/config/Official.cfg"
    #define officialView "src/minho_team/vision_node/config/Official.view"
    #define larMap "src/minho_team/vision_node/config/Lar.cfg"
    #define larVIEW "src/minho_team/vision_node/config/Lar.view"
    #define configPath "src/minho_team/vision_node/config/"
#else
    //Path definition for Qt Compilation
    #define imgPath "../localizationDaemon/Localization_Module/Images/"
    #define mainFile "../localizationDaemon/Localization_Module/Configs/main.cfg"
    #define officialMap "../localizationDaemon/Localization_Module/Configs/Official.cfg"
    #define officialView "../localizationDaemon/Localization_Module/Configs/Official.view"
    #define larMap "../localizationDaemon/Localization_Module/Configs/Lar.cfg"
    #define larVIEW "../localizationDaemon/Localization_Module/Configs/Lar.view"
    #define configPath "../localizationDaemon/Localization_Module/Configs/"
#endif

#define visParams "/visionparams.cfg"
#define lutFile "/vision.cfg"
#define maskFile "/mask.png"

#include "blackflydriver.h"

// Mathematical Defines
#define toRad (M_PI/180.0)
#define toDeg (180.0/M_PI)
#define wheelDiameter 0.1
#define deltaT 0.03  //20ms
#define CPR 3036
#define dRob 0.235
#define cost 0.5
#define theta_ 1.04719755119659774615421446109316
#define sint 0.866
#define KWheels ((M_PI*wheelDiameter)/(CPR*deltaT))

// Image Server defines
#define defaultPort 8888
#define defaultIP "127.0.0.1"
#define origGrabbing 0x01
#define segmGrabbing 0x02
#define worldGrabbing 0x03
#define histGrabbing 0x04

// Lut Defines
#define LUT_SIZE 256*256*256
#define LMIN 0
#define LMAX 1


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


struct nodo //World Map point of view
{
    float x;
    float y;
    float closestDistance;
    Point2d gradient;
};

struct localizationEstimate// Estimate of localization (vision, odometry, etc)
{
    float x;
    float y;
    float angle;
    float variance;
    QTime TimeStamp;
};

struct field// Current field definitions
{
    unsigned int FIELD_POSITIONS;
    double FIELD_WIDTH,FIELD_LENGTH;
    double HALF_FIELD_WIDTH, HALF_FIELD_LENGTH;
    float TERM1, TERM2, TERM3;
    unsigned int MAX_LINE_POINTS;
    QString FIELD_NAME;
};

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
} gameField;

typedef union fieldDimensions{
    struct gameField fieldDims;
    int dimensions[19];
} fieldDimensions;

struct hardware{
    float battery;
    float robotAngle;
    float ultraSoundSensor;
    long int enc1, enc2, enc3;
};

struct robotState
{
    Point3d robotPose;
    Point2d ballPosition;
    bool ballVisual, ballPossession;
    float ballSensorDist, ballRealDist;
    int ballID;
};

struct KalmanState{
    double x,y,thetaS, thetaC;
};

struct MTKalmanFilter{
    Point3d predictedState, predictedCovariance;
    Point3d lastCovariance, covariance;
    Point3d Q,R,K;
};

struct Pos {
    Point2d pos;
    double heading;
    double error;
};

#endif // MINHOTYPES

