#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,	//equation 42, also quiz answer
              0, 1, 0, 0;

  ekf_.F_ = MatrixXd(4, 4); //state matrix, equation 21, dt will be bultiplied for (0,2) and (1,3) in each mesurement step according to timestamp taken
  ekf_.F_ << 1, 0, 1/*dt*/, 0, //see line 81-82 for complete matrix in eq.21
             0, 1, 0, 1/*dt*/,
             0, 0, 1, 0,
             0, 0, 0, 1;

  ekf_.P_ = MatrixXd(4, 4); //state covarince matrix, mtrix is diagonal because each item is independent from other items (px-px, py-py, vx-vx and vy-vy)
  ekf_.P_ << 10, 0, 0, 0,
             0, 10, 0, 0,
             0, 0, 10, 0,
             0, 0, 0, 10;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  if (!is_initialized_) {
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) { //if first measurement is radar          
      float radial_distance = measurement_pack.raw_measurements_[0];
      //float radial_velocity = measurement_pack.raw_measurements_[2];
      float angle = measurement_pack.raw_measurements_[1];
      //convert radar measurement to cartesian coordinates
      ekf_.x_(0) = radial_distance * cos(angle);
      ekf_.x_(1) = radial_distance * sin(angle);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) { //if first measurement is lidar
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];
    }
    ekf_.x_(2) = 0; //vx = 0 for both laser(lidar) and radar
    ekf_.x_(3) = 0; //vy = 0

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  // time difference with previous measurement, updated each measurement (both radar or lidar) receive
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / (float) 1000000; 
  previous_timestamp_ = measurement_pack.timestamp_; //make previous time as current time for next iteration

  ekf_.F_(0, 2) = dt; //correct prediction matrix, F by multiplying gith delta t as in equation 21
  ekf_.F_(1, 3) = dt; //
  
  float dt_2 = dt   * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //covariance matrix, Q as in equation 40
  float n_ax = 8; //noise component, acceleration added
  float n_ay = 8; //
  
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_(0,0) = dt_4/4*n_ax;
  ekf_.Q_(0,1) = 0;
  ekf_.Q_(0,2) = dt_3/2*n_ax;
  ekf_.Q_(0,3) = 0;
  
  ekf_.Q_(1,0) = 0;
  ekf_.Q_(1,1) = dt_4/4*n_ay;
  ekf_.Q_(1,2) = 0;
  ekf_.Q_(1,3) = dt_3/2*n_ay;
  
  ekf_.Q_(2,0) = dt_3/2*n_ax;
  ekf_.Q_(2,1) = 0;
  ekf_.Q_(2,2) = dt_2*n_ax;
  ekf_.Q_(2,3) = 0;
  
  ekf_.Q_(3,0) = 0;
  ekf_.Q_(3,1) = dt_3/2*n_ay;
  ekf_.Q_(3,2) = 0;
  ekf_.Q_(3,3) = dt_2*n_ay;

  ekf_.Predict();

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {    
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_); //linearize sensor matrix with 1st degree taylor, jacobin matrix etc..
    ekf_.R_ = R_radar_; //sensor noise was already given above
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.H_ = H_laser_; //set above
    ekf_.R_ = R_laser_; //already given
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
