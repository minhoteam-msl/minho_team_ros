#include "motion.h"

Motion::Motion()
{
    previous_error = 0.0; integral = 0.0;
    acceleration_distance = 0.0; deceleration_distance = 0.0;
    acceleration_distance_aux = 0.0; distance_robotTotarget_aux = 0.0;
    acceleration_slope = 0.0, deceleration_slope = 0.0;
}

//
int Motion::angularVelocity(int target_angle, robotInfo robot, controlConfig control)
{
    int angular_velocity = 0;

    int robot_angle = (int)robot.robot_pose.z;

    float error = (float)(robot_angle - target_angle);

    if(error>180)
        error -= 360;

    gettimeofday(&stopTimerPID,NULL);
    double dt = (stopTimerPID.tv_usec - startTimerPID.tv_usec)/1000000.0;
    if(dt<0.0)
        dt = 0.03333;

    float proportionalAction = control.P * error;

    if((error*previous_error)<=0)
        integral = 0.0;

    float integralAction = control.I * integral;

    float derivative = (error-previous_error)/dt;
    float derivativeAction = control.D * derivative;

    angular_velocity = (int)(proportionalAction + integralAction + derivativeAction);

    if(angular_velocity > control.max_angular_velocity)
        angular_velocity = control.max_angular_velocity;
    else if(angular_velocity < -control.max_angular_velocity)
        angular_velocity = -control.max_angular_velocity;
    else
        integral += error * dt;

    previous_error = error;

    //ROS_INFO("proport: %f  integ: %f  deriv: %f\n",proportionalAction,integralAction, derivativeAction);

    gettimeofday(&startTimerPID,NULL);

    return angular_velocity;
}

//
int Motion::movementDirection(robotInfo robot, int target_angle)
{
    int movement_direction = 360 - (int)robot.robot_pose.z + target_angle;

    if(movement_direction >= 360)
        movement_direction -= 360;

    return movement_direction;
}

//
/*
 * (velocity)
 *   |       ___________________
 *   |      /                   \
 *   |     /                     \
 *   |    /                       \
 *   |_ _/_ _ _ _ _ _ _ _ _ _ _ _ _\_ _ _ _(distance)
 *
 */
//
int Motion::linearVelocity_slope(float distance_robotTotarget, controlConfig control, bool new_target)
{
    if(new_target) {
        ROS_INFO("dist: %f cacc: %f\n",distance_robotTotarget,control.acceleration);
        acceleration_distance = control.acceleration * distance_robotTotarget;
        deceleration_distance = control.deceleration * distance_robotTotarget;
        acceleration_slope = ((float)control.max_linear_velocity)/acceleration_distance;
        deceleration_slope = ((float)control.max_linear_velocity)/deceleration_distance;
        acceleration_distance_aux = distance_robotTotarget - acceleration_distance;
        distance_robotTotarget_aux = distance_robotTotarget;
        return 10;
    }

    if(distance_robotTotarget>deceleration_distance && distance_robotTotarget<acceleration_distance_aux)
        return control.max_linear_velocity;
    else if(distance_robotTotarget>acceleration_distance_aux)
        return (int)(-acceleration_slope * (distance_robotTotarget - distance_robotTotarget_aux));
    else
        return (int)(deceleration_slope * distance_robotTotarget);
}


//tanh((x-10)*(1/2))*tanh((-x-0)*(1/2))*80

