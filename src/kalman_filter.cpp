#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &Rl_in, MatrixXd &Hj_in, MatrixXd &Rr_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Hj_ = Hj_in;
  Rr_ = Rr_in;
  Rl_ = Rl_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + Rl_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

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
	std::cout << "Hj: " << Hj_ << std::endl << std::endl;
	std::cout << "Rr: " << Rr_ << std::endl << std::endl;
	std::cout << "x: " << x_ << std::endl << std::endl;

	float ro = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
	if (ro == 0) {
		ro = 0.0001;		
	}

	float phi = atan2(x_(1), x_(0));
	float ro_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / ro;


	VectorXd h = VectorXd(3);
	h << ro, phi, ro_dot;	
	
	VectorXd y = z - h;

	if (y(1) > 2 * PI_) {
		y(1) = y(1) - (2 * PI_);
	}
	if (y(1) < -2 * PI_) {
		y(1) = y(1) + 2 * PI_;
	}

	Hj_ =  t.CalculateJacobian(x_);

	MatrixXd Hjt = Hj_.transpose();
	MatrixXd S = Hj_ * P_ * Hjt + Rr_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Hjt;
	MatrixXd K = PHt * Si;

	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj_) * P_;

}
