#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  x_ = F_ * x_; //equation 11 in sensor-fusion-ekf-reference.pdf
  P_ = F_ * P_ * F_.transpose() + Q_; //equation 12 in sensor-fusion-ekf-reference.pdf
}

//used for lidar (laser) sensor, H is already linear
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;   //eqaution 13, y is error between best prediction (mean) and measurement
  MatrixXd S = H_ * P_ * H_.transpose() + R_; //equation 14
  MatrixXd K = P_ * H_.transpose() * S.inverse(); //equation 15, kalman gain

  x_ = x_ + (K * y);  //eqution 16, new state
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_; //equation 17, new uncertinity
}

//used for radar sensor (linearized using jacobian m)
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float radial_distance, angle, radial_velocity;
  radial_distance = sqrt(x_(0)*x_(0) + x_(1)*x_(1)); //equation 54 in sensor-fusion-ekf-reference.pdf
  angle = atan2(x_(1), x_(0)); //equation 55
  radial_velocity = (x_(0)*x_(2) + x_(1)*x_(3)) / radial_distance; //equation 59 - todo: div by zero
  
  VectorXd H_x = VectorXd(3);
  H_x(0) = radial_distance;
  H_x(1) = angle;
  H_x(2) = radial_velocity;
  
  //difference between measurement(z) and prediction (H_x) - linearized with first degree taylor series (previously using jacobian matrix)
  VectorXd y = z - H_x;//equation 13 
  
  while(y(1) > M_PI){ //normalization is required, otherwise I had high RMSE in the end of tour in simultion
    y(1) -= 2*M_PI;
  }
  
  //todo: common process with the laser update function
  MatrixXd S = H_ * (P_ * H_.transpose()) + R_;//equation 14
  MatrixXd K = (P_ * H_.transpose()) * S.inverse();//equation 15

  x_ = x_ + (K * y);//eqution 16, new state estimation
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;//equation 17, new uncertinity
}
