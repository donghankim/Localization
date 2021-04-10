#include <iostream>
#include "kalman_filter.h"
#define PI 3.14159265


using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;

}

// LiDAR
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_new = H_*x_;
  VectorXd y = z - z_new;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_*Ht*S_inv;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  x_ = x_ + (K*y);
  P_ = (I-K*H_)*P_;
}

// RADAR
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float x_0 = x_(0);
  float x_1 = x_(1);
  float x_2 = x_(2);
  float x_3 = x_(3);

  if(x_0 == 0 && x_1 == 0)
    return;

  float rho = sqrt(pow(x_0, 2) + pow(x_1, 2));
  float theta = atan2(x_1, x_0);
  float rho_dot;
  
  if(fabs(rho) < 0.0001)
    rho_dot = 0;
  else
    rho_dot = (x_0*x_2 + x_1*x_3)/rho;
  
  VectorXd H = VectorXd(3);
  H << rho, theta, rho_dot;
  VectorXd y = z - H;
  y(1) = atan2(sin(y(1)), cos(y(1)));
  /*
  if(y(1) > PI)
    y(1) -= 2.f*PI;
  if(y(1) < -PI)
    y(1) += 2.f*PI;
  */
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_*Ht*S_inv;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  x_ = x_ + (K*y);
  P_ = (I-K*H_)*P_;
}
