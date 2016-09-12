#ifndef TYPES_H
#define TYPES_H

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

#endif // TYPES_H
