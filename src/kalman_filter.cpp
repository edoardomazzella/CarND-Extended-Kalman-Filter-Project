#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;


/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(const VectorXd &x_in, const MatrixXd &P_in, const MatrixXd &F_in,
                        const MatrixXd &H_in, const VectorXd &hx_in, const MatrixXd &Hj_in, 
                        const MatrixXd &Q_in, const MatrixXd &R_laser_in, const MatrixXd &R_radar_in) { 
  
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  hx_ = hx_in;
  Hj_ = Hj_in;
  Q_ = Q_in;
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;
}

void KalmanFilter::Init(VectorXd &&x_in, MatrixXd &&P_in, MatrixXd &&F_in,
                        MatrixXd &&H_in, VectorXd &&hx_in, MatrixXd &&Hj_in,
                        MatrixXd &&Q_in, MatrixXd &&R_laser_in, MatrixXd &&R_radar_in) {
  
  x_ = std::move(x_in);
  P_ = std::move(P_in);
  F_ = std::move(F_in);
  H_ = std::move(H_in);
  hx_ = std::move(hx_in);
  Hj_ = std::move(Hj_in);
  Q_ = std::move(Q_in);
  R_laser_ = std::move(R_laser_in);
  R_radar_ = std::move(R_radar_in);
}

void KalmanFilter::Predict() {
  
  // Mean
  x_ = F_ * x_;
  // Covariance
  P_ = F_ * P_ * F_.transpose() + Q_;
  
  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Calculate y using non linear h
  VectorXd y = z - hx_;
  tools::NormalizeAngle(y);
  
  // Calculate jacobian and use it instead of H_ for S, K and P
  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Hjt + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Hjt;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj_) * P_;
  
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}
