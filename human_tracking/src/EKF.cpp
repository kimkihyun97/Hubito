#include "EKF.h"

EKF::EKF() 
{
    x.setZero();
    P.setIdentity();
    u.setZero();

    Q<< 0.2,0.0,0.0,0.0,0.2,0.0,0.0,0.0,0.21; // set this to your process noise
    R << 7.961199e-05, 0.0 ,0.0 ,0.0 ,3.5253 ,0.0 ,0.0 ,0.0, 7.4579e-06; // set this to your measurement noise
    z.setZero();
}

void EKF::calculateJacobian(float dt) 
{

    F.setIdentity();
    F(0, 2) = -u(0)*dt*sin(x(2) + (u(1)*dt/2));
    F(1, 2) = u(0)*dt*cos(x(2) + (u(1)*dt/2));
}

void EKF::predict(float dt) 
{
    // predict state
    x(0) += u(0)*dt*cos(x(2) + (u(1)*dt/2));
    x(1) += u(0)*dt*sin(x(2) + (u(1)*dt/2));
    x(2) += u(1)*dt; // radian
    if (x(2) > 2 * M_PI) 
    {
      x(2) -= 2.0 * M_PI;
    } 
    else if (x(2) < -2 * M_PI) 
    {
      x(2) += 2.0 * M_PI;
    }
    // predict covariance
    calculateJacobian(dt);
    P = F*P*F.transpose() + Q;
}

void EKF::update(float hz, float hx) 
{
    // measurement update
    float dis_predict = sqrt(pow((hz), 2) + pow((hx), 2));
    Eigen::Vector3f Hx;

    Hx << dis_predict, atan2(hx, hz),x(2);
    
    Eigen::Vector3f y = z - Hx;
    
    Hj.setZero();
    Hj(0, 0) = (-hz) / dis_predict;
    Hj(0, 1) = (-hx) / dis_predict;
    Hj(1, 0) = (hz) / pow(dis_predict, 2);
    Hj(1, 1) = (-hx) / pow(dis_predict, 2);
    Hj(2, 2) = 1;
    Eigen::Matrix3f S = Hj*P*Hj.transpose() + R;
    Eigen::Matrix<float, 3, 3> K = P*Hj.transpose()*S.inverse();

    x += K*y;
    if(u(0) == 0.0 && u(1) == 0.0)
    {
      x.setZero();
    }
    P = (Eigen::Matrix3f::Identity() - K*Hj)*P;
}