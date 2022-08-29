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
  //n,m
  double dt = 0.1;
  int n = 3;
  int m = 2;
  auto kf = setup_KalmanFilter(n,m,dt);

   // List of noisy position measurements (y)
  std::vector<double> measurements = {
      1.04202710058, 1.10726790452, 1.2913511148, 1.48485250951, 1.72825901034,
      1.74216489744, 2.11672039768, 2.14529225112, 2.16029641405, 2.21269371128,
      2.57709350237, 2.6682215744, 2.51641839428, 2.76034056782, 2.88131780617,
      2.88373786518, 2.9448468727, 2.82866600131, 3.0006601946, 3.12920591669,
      2.858361783, 2.83808170354, 2.68975330958, 2.66533185589, 2.81613499531,
      2.81003612051, 2.88321849354, 2.69789264832, 2.4342229249, 2.23464791825,
      2.30278776224, 2.02069770395, 1.94393985809, 1.82498398739, 1.52526230354,
      1.86967808173, 1.18073207847, 1.10729605087, 0.916168349913, 0.678547664519,
      0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013, -0.602973173813
  };

  //Initialization t = 0
  // Best guess of initial states
  Eigen::VectorXd x0(n);
  double t = 0;
  x0 << measurements[0], 0, -9.81;
  kf->init(t, x0);

  Eigen::VectorXd y(m);


  // Feed measurements into filter, output estimated states  
  std::cout << "t = " << t << ", " << "x_hat[0]: " << kf-> state().transpose() << std::endl;

  for(int i = 0; i < measurements.size(); i++) {
    t += dt;
    y << measurements[i],measurements[i];
    kf->update(y);
    std::cout << "t = " << t << ", " << "y[" << i << "] = " << y.transpose()
        << ", x_hat[" << i << "] = " << kf->state().transpose() << std::endl;
  }
  return 0;
}
