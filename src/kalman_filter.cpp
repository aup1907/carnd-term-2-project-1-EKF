#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * Code from lesson
  */
    x_ = F_ * x_; //+ u;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * Code from lesson
  */
    VectorXd y = z- H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}


void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    float theta = atan2(x_(1), x_(0));
    
    if (fabs(rho) < .00001) {
        //division by zero check
        rho = sqrt(.00002);
    }
    
    float rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;

    
    VectorXd h = VectorXd(3); // h(x_)
    h << rho, theta, rho_dot;
    
    VectorXd y = z - h;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;
    
    const double Max = M_PI;
    const double Min = -M_PI;
    
    y(1) = y(1) < Min
    ? Max + std::fmod(y(1) - Min, Max - Min)
    : std::fmod(y(1) - Min, Max - Min) + Min;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;

    
}
