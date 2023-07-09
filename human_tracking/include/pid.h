#ifndef PID_H
#define PID_H

struct PID_Coef {
    float Kp=0.6;
    float Ki=0.12;
    float Kd=0.1;
    float Ek;
    float Ek1;
    float Ek2;
};

extern PID_Coef ANGLE_PID;
extern PID_Coef DIS_PID;
extern PID_Coef Left_PID;
extern PID_Coef right_PID;

float PID_control(float SetValue, float ActualValue, PID_Coef *PID);
#endif
