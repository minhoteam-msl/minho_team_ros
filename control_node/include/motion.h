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
    Motion(Fundamental *fund);
    int angularVelocity_PID(float robot_angle, float reference_angle, controlConfig cconfig);
    int movementDirection(int robot_angle, int reference_angle);

    int linearVelocity(controlConfig cconfig, const vector<Point>& path, int ai_action, float percent_vel);


private:
    int linearVelocity_PID(float error, controlConfig cconfig, int max_linear_velocity);

    Fundamental *fundamental;
    //PID angular velocity
    float rot_previous_error, rot_integral;
    struct timeval rot_start_timer, rot_stop_timer;
    //PID linear velocity
    float lin_previous_error, lin_integral;
    struct timeval lin_start_timer, lin_stop_timer;

};

#endif // MOTION_H
