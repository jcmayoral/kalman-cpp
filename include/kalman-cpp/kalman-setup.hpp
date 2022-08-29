/**
 * Test for the KalmanFilter class with 1D projectile motion.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>

#include <kalman-cpp/kalman.hpp>

using namespace kalman;


//int n = 3; // Number of states
//int m = 1; // Number of measurements
std::shared_ptr<KalmanFilter> setup_KalmanFilter(int n, int m,double dt =0.1) {
  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
  C << 1, 0, 0, 1,0,0;

  // Reasonable covariance matrices
  Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
  std::cout << "A " << std::endl;
  R << 10,0.5,0.5,10;

  P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;
  // Construct the filter
  return std::make_shared<KalmanFilter>(*(new KalmanFilter(dt,A, C, Q, R, P)));
}
