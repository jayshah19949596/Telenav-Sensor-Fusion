#include "measurement_package.h"
#include "kalman_filter.h"
#include <iostream>
// #include "Eigen"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter()
{

  dt = 0.1;
  is_initialized_ = false;
  previous_timestamp_ = 0;

  F_ = MatrixXd(2, 2);
  F_ << 1, dt,
        0, 1;

  x_ = VectorXd(2);
  x_.fill(0);

  Q_ = MatrixXd(2, 2);
  Q_ << 0.01, 0,
        0, 0.01;
//  Q_ << noise_ax*noise_ax, 0,
//      0, noise_ay*noise_ay;



  H_ = MatrixXd(2, 2);
  H_ << 1, 0,
        0, 1;

  R_ = MatrixXd(2, 2);
  R_ << 0.01, 0,
        0, 0.01;

//  R_ = MatrixXd(1, 1);
//  R_ << 1;


  P_ = MatrixXd(2, 2);
  P_ << 100, 0,
        0, 100;

}


KalmanFilter::~KalmanFilter() {}


void KalmanFilter::perform_kf(const MeasurementPackage &measurement_pack)
{
  if (!is_initialized_)
  {

    // first measurement
    x_ = VectorXd(2);
    x_ << 1, 1;
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1];
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1];
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
//  dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  this->Predict();
  this->Update(measurement_pack.raw_measurements_);
}

void KalmanFilter::Predict()
{
  // =======================
  // predicting the state
  // =======================
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft ;


}

void KalmanFilter::Update(const VectorXd &z) {
  // ===================================================
  // update the state by using Kalman Filter equations
  // ===================================================
  VectorXd z_predicted = H_ * x_;
  VectorXd y = z - z_predicted;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  // ==============
  //  new estimate
  // ==============
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
