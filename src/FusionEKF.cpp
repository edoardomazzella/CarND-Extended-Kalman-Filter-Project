#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  
  // create a 4D state vector, we don't know yet the values of the x state
  VectorXd x = VectorXd(4);
  
  // state covariance matrix P
  MatrixXd P = MatrixXd(4, 4);
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1000, 0,
       0, 0, 0, 1000;
  
  // the initial transition matrix F_
  MatrixXd F = MatrixXd(4, 4);
  F << 1, 0, 1, 0,
       0, 1, 0, 1,
       0, 0, 1, 0,
       0, 0, 0, 1;
  
  // linear measurement matrix
  MatrixXd H = MatrixXd(2, 4);
  H << 1, 0, 0, 0,
       0, 1, 0, 0;
  
  VectorXd hx = VectorXd(3);
  
  // H Jacobian initialization
  MatrixXd Hj = MatrixXd(3, 4);
  
  // Process noise covariance matrix
  MatrixXd Q = MatrixXd(4, 4);
  
  // measurement covariance matrix - laser
  MatrixXd R_laser = MatrixXd(2, 2);
  R_laser << 0.0225, 0,
  			 0, 0.0225;
  
  // measurement covariance matrix - radar
  MatrixXd R_radar = MatrixXd(3, 3);
  R_radar << 0.09, 0, 0,
             0, 0.0009, 0,
             0, 0, 0.09;  
  
  ekf_.Init(std::move(x), std::move(P), std::move(F), 
            std::move(H), std::move(hx), std::move(Hj), 
            std::move(Q), std::move(R_laser), std::move(R_radar));
  
  // set the acceleration noise components
  noise_ax_ = 9;
  noise_ay_ = 9;
  
  // set other private members
  is_initialized_ = false;
  previous_timestamp_ = 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;
    
    VectorXd x = VectorXd(4);
    // set the state with the initial location and zero velocity
    if (MeasurementPackage::RADAR == measurement_pack.sensor_type_) {
      cout << endl << "First iteration - Radar" << endl;
      // Convert radar from polar to cartesian coordinates 
      // and initialize state.
      x = tools::Polar2Cartesian(measurement_pack.raw_measurements_);
    }
    else if (MeasurementPackage::LASER == measurement_pack.sensor_type_) {
      cout << endl << "First iteration - Laser" << endl;
      // Initialize state.
      x << measurement_pack.raw_measurements_[0], 
           measurement_pack.raw_measurements_[1], 
           0, 
           0;
    }
    else {
      cout << endl << "Error: invalid sensor type." << endl;
      return;
    }
    ekf_.SetX(std::move(x));

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    
    cout << "x_ = " << ekf_.GetX() << endl;
    cout << "P_ = " << ekf_.GetP() << endl;
  }
  else {
  	   /**
  	    * Prediction
  	    */
	
  	   // Update the state transition matrix F according to the new elapsed time.
  	   // Time is measured in seconds.
  	   double elapsed_time = ((double) measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  	   previous_timestamp_ = measurement_pack.timestamp_;
    
       cout << "Elapsed time: " <<  elapsed_time << endl;
  	
  	   MatrixXd F = MatrixXd(4, 4);
  	   F << 1,     0, elapsed_time,            0,
  	        0,     1,            0, elapsed_time,
  	        0,     0,            1,            0,
  	        0,     0,            0,            1;
  	   ekf_.SetF(std::move(F));
  	
  	   // Update the process noise covariance matrix.
  	   // Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  	   ekf_.SetQ(tools::CalculateProcessNoiseCovarianceMatrix(elapsed_time, noise_ax_, noise_ay_));
	
  	   cout << endl << "Predict step." << endl;
  	   ekf_.Predict();
	
  	   /**
  	    * Update
  	    */
  	   if (MeasurementPackage::RADAR == measurement_pack.sensor_type_) {
  	     cout << endl << "Radar update step." << endl;
  	     VectorXd x_state = ekf_.GetX();
  	  
  	     // Radar updates
  	     ekf_.Sethx(tools::NonLinearH(x_state));
  	     ekf_.SetHj(tools::CalculateJacobian(x_state));
  	  
  	     ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  	   } 
  	   else if (MeasurementPackage::LASER == measurement_pack.sensor_type_) {
  	     cout << endl << "Laser update step." << endl;
  	     // Laser updates
  	     ekf_.Update(measurement_pack.raw_measurements_);
  	   }
  	   else {
  	     cout << endl << "Error: invalid sensor type." << endl;
  	     return;
       }
  }
}
