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
 
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;	
	MatrixXd S = H_ * P_ * H_.transpose() + Rl_;		
	MatrixXd K = P_ * H_.transpose() * S.inverse();

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;	
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {


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

	Hj_ =  tools.CalculateJacobian(x_);
	
	MatrixXd S = Hj_ * P_ * Hj_.transpose() + Rr_;		
	MatrixXd K = P_ * Hj_.transpose() * S.inverse();

	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj_) * P_;

}
