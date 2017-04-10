#include "motion.h"

Motion::Motion(Fundamental *fund)
{
    fundamental = fund;

    rot_previous_error = 0.0; rot_integral = 0.0;
    lin_previous_error = 0.0; lin_integral = 0.0;
}

//
int Motion::angularVelocity_PID(float robot_angle, float reference_angle, controlConfig cconfig)
{
    int angular_velocity = 0;

    float error = robot_angle - reference_angle;

    if(error>180.0)
        error -= 360.0;

    gettimeofday(&rot_stop_timer,NULL);
    float dt = (rot_stop_timer.tv_usec - rot_start_timer.tv_usec)/1000000.0;
    if(dt<0.0)
        dt = 0.03333;

    float proportionalAction = cconfig.Kp_rot * error;

    if((error*rot_previous_error)<=0)
        rot_integral = 0.0;

    float integralAction = cconfig.Ki_rot * rot_integral;

    float derivative = (error-rot_previous_error)/dt;
    float derivativeAction = cconfig.Kd_rot * derivative;

    angular_velocity = (int)(proportionalAction + integralAction + derivativeAction);

    if(angular_velocity > cconfig.max_angular_velocity)
        angular_velocity = cconfig.max_angular_velocity;
    else if(angular_velocity < -(int)cconfig.max_angular_velocity)
        angular_velocity = -(int)cconfig.max_angular_velocity;
    else
        rot_integral += error * dt;

    rot_previous_error = error;

    gettimeofday(&rot_start_timer,NULL);

    return angular_velocity;
}

//
int Motion::linearVelocity_PID(float error, controlConfig cconfig, int max_linear_velocity)
{
    //cout<<"Kp_lin: "<<cconfig.Kp_lin<<" Ki_lin: "<<cconfig.Ki_lin<<" Kd_lin: "<<cconfig.Kd_lin<<endl;
    //cout<<"max_linear_velocity: "<<max_linear_velocity<<endl;
    //cout<<"error: "<<error<<endl;

    int linear_velocity = 0;

    gettimeofday(&lin_stop_timer,NULL);
    float dt = (lin_stop_timer.tv_usec - lin_start_timer.tv_usec)/1000000.0;
    if(dt<0.0)
        dt = 0.03333;

    float proportionalAction = cconfig.Kp_lin * error;

    float integralAction = cconfig.Ki_lin * lin_integral;

    float derivative = (error-lin_previous_error)/dt;
    float derivativeAction = cconfig.Kd_lin * derivative;

    linear_velocity = (int)(proportionalAction + integralAction + derivativeAction);

    if(linear_velocity > max_linear_velocity)
        linear_velocity = max_linear_velocity;
    else
        lin_integral += error * dt;

    lin_previous_error = error;

    gettimeofday(&lin_start_timer,NULL);

    return linear_velocity;
}

//
int Motion::linearVelocity(controlConfig cconfig, const vector<Point>& path, int ai_action, float percent_vel)
{
    int linear_velocity = 0;
    float dist_seg = 0.0;
    float dist_total = 0.0;
    float error = 0.0;
    float dist_robot_ball = 0.0;
    int max_linear_velocity = 0;

    if(path.size() > 1) {

        for(int i=0; i<(int)path.size()-1; i++) {

            dist_seg = fundamental->distance(path.at(i).x(), path.at(i).y(), path.at(i+1).x(), path.at(i+1).y());

            dist_total += dist_seg;
            //cout<<"seg: "<<i<<" : "<<i+1<<"  dist_seg_path: "<<dist_seg<<endl;
        }
        //cout<<endl;
        //cout<<"dist_total: "<<dist_total<<endl<<endl;
    }

    if(ai_action==aENGAGEBALL)
        dist_robot_ball = 0.25;

    error = dist_total-dist_robot_ball;
    if(error<0.0)
        error = 0.0;

    max_linear_velocity = cconfig.max_linear_velocity * percent_vel;

    linear_velocity = linearVelocity_PID(error, cconfig, max_linear_velocity);

    //cout<<"lin_vel: "<<linear_velocity<<endl<<endl;

    return linear_velocity;
}

//
/*
int Motion::linearVelocity(robotInfo robot, controlConfig cconfig, const vector<Point>& path, int& movement_direction)
{
    int linear_velocity = 0;
    float dist_seg = 0.0;
    float dist_total = 0.0;
    float error = 0.0;
    int reference_angle = 0;

    if(path.size() > 1) {

        reference_angle = fundamental->cartesian2polar_angleDegNormalize(path.at(0).x(), path.at(0).y(),
                                                                      path.at(1).x(), path.at(1).y());
        movement_direction = movementDirection((int)robot.robot_pose.z, reference_angle);

        for(int i=0; i<(int)path.size()-1; i++) {

            dist_seg = fundamental->distance(path.at(i).x(), path.at(i).y(), path.at(i+1).x(), path.at(i+1).y());

            dist_total += dist_seg;
            //cout<<"seg: "<<i<<" : "<<i+1<<"  dist_seg_path: "<<dist_seg<<endl;
        }
        //cout<<endl;
        //cout<<"dist_total: "<<dist_total<<endl<<endl;

    }

    error = dist_total-0.25;
    if(error<0.0)
        error = 0.0;

    linear_velocity = linearVelocity_PID(error, cconfig);

    //cout<<"lin_vel: "<<linear_velocity<<endl<<endl;

    return linear_velocity;
}
*/

//
int Motion::movementDirection(int robot_angle, int reference_angle)
{
    int movement_direction = 360 - robot_angle + reference_angle;

    if(movement_direction >= 360)
        movement_direction -= 360;

    return movement_direction;
}



//tanh((x-10)*(1/2))*tanh((-x-0)*(1/2))*80

