//
// Created by zm on 18-12-1.
//

#ifndef OFFB_POSCTL_PARAMETER_H
#define OFFB_POSCTL_PARAMETER_H


class Parameter {

public:
    float pos_x;
    float pos_y;
    float pos_z;

    float x_p;
    float y_p;
    float z_p;

    float vx_p;
    float vy_p;
    float vz_p;

    float vx_i;
    float vy_i;
    float vz_i;

    float vx_d;
    float vy_d;
    float vz_d;

    float vx_error_p;
    float vy_error_p;
    float vz_error_p;

    float vx_error_i;
    float vy_error_i;
    float vz_error_i;

    float vx_error_d;
    float vy_error_d;
    float vz_error_d;

    //pix_controller_thrust PID parameter in Z axis
    float error_z_pos_P;
    float error_z_pos_I;
    float error_z_pos_D;

    float error_z_vel_P;
    float error_z_vel_I;
    float error_z_vel_D;

    bool readParam(const char* addr);

};


#endif //OFFB_POSCTL_PARAMATER_H
