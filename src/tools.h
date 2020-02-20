#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

namespace tools {
     /**
      * Helper function to normalize y angle between pi and -pi
      */
     void NormalizeAngle(Eigen::VectorXd &y);
     /**
      * A helper method to map the state vector in polar coordinates
      */
     Eigen::VectorXd NonLinearH(const Eigen::VectorXd &x_state);
     
     /**
      * A helper method to transform the polar radar measurements in cartesian
      */
     Eigen::VectorXd Polar2Cartesian(const Eigen::VectorXd &raw_measurements);
   
     /**
      * A helper method to calculate the process noise covariance matrix.
      */
     Eigen::MatrixXd CalculateProcessNoiseCovarianceMatrix(double elapsed_time, 
                                                           double noise_ax, double noise_ay);
     
     /**
      * A helper method to calculate Jacobians.
      */
     Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);
     
     /**
      * A helper method to calculate RMSE.
      */
     Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                   const std::vector<Eigen::VectorXd> &ground_truth);
}

#endif  // TOOLS_H_
