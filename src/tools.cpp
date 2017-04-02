#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
	  std::cout << "Estimations and groud truth vectors must have the same size." << std::endl;
	  return rmse;
  }


  for (int i = 0; i < estimations.size(); ++i) {
	  VectorXd diff = (estimations[i] - ground_truth[i]);
	  diff = diff.array() * diff.array();
	  rmse = rmse + diff;
  }
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
	  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	const float SMALL_FLT = 0.0001;
	MatrixXd Hj = MatrixXd(3, 4);
  //recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
	if (px == 0 && py == 0) {		
		px = SMALL_FLT;
		py = SMALL_FLT;
	}
	//compute the Jacobian matrix
	float denum = px * px + py * py;
	Hj << px / pow(denum, 0.5), py / pow(denum, 0.5), 0, 0,
		-py / denum, px / denum, 0, 0,
		py*(vx*py - vy*px) / pow(denum, 1.5), px*(vy*px - vx*py) / pow(denum, 1.5), px / pow(denum, 0.5), py / pow(denum, 0.5);


	return Hj;
}
