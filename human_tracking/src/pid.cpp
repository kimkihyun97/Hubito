#include "pid.h"

PID_Coef ANGLE_PID;
PID_Coef DIS_PID;
PID_Coef Left_PID;
PID_Coef right_PID;

float PID_control(float SetValue, float ActualValue, PID_Coef *PID) {
    float PIDcontrol;

    PID->Ek = SetValue - ActualValue;
    PIDcontrol = (PID->Kp * PID->Ek) - (PID->Ki * PID->Ek1) + (PID->Kd * PID->Ek2);

    PID->Ek2 = PID->Ek1;
    PID->Ek1 = PID->Ek;

    return PIDcontrol;
}