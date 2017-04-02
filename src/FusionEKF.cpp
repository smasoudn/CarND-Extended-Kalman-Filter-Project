#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
	         0, 1, 0, 1,
	         0, 0, 1, 0,
	         0, 0, 0, 1;
  
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
       	  0, 1, 0, 0,
	      0, 0, 1000, 0,
	      0, 0, 0, 1000;
  
  //set the acceleration noise components
  noise_ax_ = 9.0;
  noise_ay_ = 9.0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

	ekf_.Q_ = MatrixXd(4, 4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
		float ro = measurement_pack.raw_measurements_(0);
		float phi = measurement_pack.raw_measurements_(1);
		float ro_dot = measurement_pack.raw_measurements_(2);
		ekf_.x_ << ro * cos(phi), ro * sin(phi), ro_dot * cos(phi), ro_dot * sin(phi);		
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
		ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;		
    }

    // done initializing, no need to predict or update
	previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
 
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
   previous_timestamp_ = measurement_pack.timestamp_;
   
   ekf_.F_(0, 2) = dt;
   ekf_.F_(1, 3) = dt;

   // Set the process covariance matrix Q  
   ekf_.Q_ << pow(dt, 4) * noise_ax_ / 4, 0, pow(dt, 3) * noise_ax_ / 2, 0,
	          0, pow(dt, 4) * noise_ay_ / 4, 0, pow(dt, 3) * noise_ay_ / 2,
	          pow(dt, 3) * noise_ax_ / 2, 0, pow(dt, 2) * noise_ax_, 0,
	          0, pow(dt, 3) * noise_ay_ / 2, 0, pow(dt, 2) * noise_ay_;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	  

  } else {
    // Laser updates
	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
