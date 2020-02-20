#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(const Eigen::VectorXd &x_in, const Eigen::MatrixXd &P_in, const Eigen::MatrixXd &F_in,
            const Eigen::MatrixXd &H_in, const Eigen::VectorXd &hx_in, const Eigen::MatrixXd &Hj_in, 
            const Eigen::MatrixXd &Q_in, const Eigen::MatrixXd &R_laser_in, const Eigen::MatrixXd & R_radar_in);
  
  void Init(Eigen::VectorXd &&x_in, Eigen::MatrixXd &&P_in, Eigen::MatrixXd &&F_in,
            Eigen::MatrixXd &&H_in, Eigen::VectorXd &&hx_in, Eigen::MatrixXd &&Hj_in, 
            Eigen::MatrixXd &&Q_in, Eigen::MatrixXd &&R_laser_in, Eigen::MatrixXd &&R_radar_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);
  
  // Getter functions
  const Eigen::VectorXd &GetX() const { return x_; }
  const Eigen::MatrixXd &GetP() const { return P_; }
  
  // Setter functions
  void SetX(const Eigen::VectorXd &x_in) { x_ = x_in; }
  void SetX(Eigen::VectorXd &&x_in) { x_ = std::move(x_in); }
  
  void SetF(const Eigen::MatrixXd &F_in) { F_ = F_in; }
  void SetF(Eigen::MatrixXd &&F_in) { F_ = std::move(F_in); }
  
  void Sethx(const Eigen::VectorXd &hx_in) { hx_ = hx_in; }
  void Sethx(Eigen::VectorXd &&hx_in) { hx_ = std::move(hx_in); }
  
  void SetHj(const Eigen::MatrixXd &Hj_in) { Hj_ = Hj_in; }
  void SetHj(Eigen::MatrixXd &&Hj_in) { Hj_ = std::move(Hj_in); }
  
  void SetQ(const Eigen::MatrixXd &Q_in) { Q_ = Q_in; }
  void SetQ(Eigen::MatrixXd &&Q_in) { Q_ = std::move(Q_in); }
  
private:
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // linear measurement matrix
  Eigen::MatrixXd H_;
  
  // non linear h(x)
  Eigen::VectorXd hx_;
  // H Jacobian
  Eigen::MatrixXd Hj_;
  
  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement covariance matrixes
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
};

#endif // KALMAN_FILTER_H_
