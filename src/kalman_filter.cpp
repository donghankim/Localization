#include "kalman_filter.h"

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
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_new = H_*x_;
  VectorXd y = z - H_*x_;
  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_*H_.transpose()*S.inverse();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I-K*H_)*P_;
  x_ += K*y;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  double x_1 = x_(0);
  double x_2 = x_(1);
  double x_4 = x_(3);
  double rho = sqrt(pow(x_1, 2) + pow(x_2, 2));
  double theta = atan2(x_2, x_1);
  double rho_dot;
  
  if(fabs(rho) < 0.0001)
    rho_dot = 0;
  else
    rho_dot = (x_1*x_2 + x_2*x_4)/rho;
  
  VectorXd H = VectorXd(3);
  H << rho, theta, rho_dot;
  VectorXd y = z - H;

  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_*H_.transpose()*S.inverse();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I-K*H_)*P_;
  x_ += K*y;
}
