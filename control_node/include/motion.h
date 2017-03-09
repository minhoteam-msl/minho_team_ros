/*
 * LAR - Laboratório de Automação e Robótica
 * MinhoTeam robotic soccer team
 * http://www.robotica.dei.uminho.pt/robocup2016/
 * DEI - University of Minho, Portugal
 *
 * motion.h
 *
 * Created: ,2016
 * Author: Tiago Maia
 *
 */

#ifndef MOTION_H
#define MOTION_H

//
#include "fundamental.h"


class Motion
{
public:
    Motion();
    int angularVelocity(int target_angle, robotInfo robot, controlConfig control);
    int movementDirection(robotInfo robot, int target_angle);

    int linearVelocity_slope(float distance_robotTotarget, controlConfig control, bool new_target);


private:
    //PID Angular Velocity
    float previous_error, integral;
    struct timeval startTimerPID, stopTimerPID;
    //Linear Velocity Slope
    float acceleration_distance, deceleration_distance;
    float acceleration_distance_aux, distance_robotTotarget_aux;
    float acceleration_slope, deceleration_slope;

};

#endif // MOTION_H
