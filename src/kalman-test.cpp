/**
 * Test for the KalmanFilter class with 1D projectile motion.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <kalman-cpp/kalman-setup.hpp>

using namespace kalman;

int main(int argc, char* argv[]) {
  auto kf = setup_KalmanFilter();
  measure(kf);
  return 0;
}
