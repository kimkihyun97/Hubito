#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include <cmath>

class EKF 
{
public:
    EKF();

    void predict(float dt);
    void update(float z, float x);

    Eigen::Vector3f x; // state vector: x, z, theta
    Eigen::Matrix3f P; // state covariance matrix
    Eigen::Vector2f u; // control input: v, w
    void calculateJacobian(float dt);

    Eigen::Matrix3f F; // Jacobian of the state transition function
    Eigen::Matrix3f Q; // process noise covariance
    Eigen::Matrix3f R; // measurement noise covariance
    Eigen::Vector3f z; // sensor measurement
    Eigen::Matrix3f Hj; // observation model
};

#endif // EKF_H
