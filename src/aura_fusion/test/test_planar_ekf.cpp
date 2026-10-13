#include "aura_fusion/planar_ekf.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

namespace af = aura_fusion;
void require(bool condition, const char* message)
{
  if (!condition) { throw std::runtime_error(message); }
}
void covarianceValid(const af::PlanarEKF& filter)
{
  const auto& P = filter.covariance();
  require(P.allFinite() && P.isApprox(P.transpose(), 1e-10), "Covariance finite/symmetric");
  Eigen::SelfAdjointEigenSolver<af::Covariance> eig(P);
  require(eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() >= -1e-10,
    "Covariance positive semidefinite");
}

int main()
{
  try {
    const auto R = af::wheelCovariance(0.0325, 0.21, 0.1, 0.1);
    require(std::abs(af::wheelTwist(3.077, 3.077, 0.0325, 0.21)(0) - 0.1000025) < 1e-12,
      "Wheel units/signs");
    const auto unequal_R = af::wheelCovariance(0.0325, 0.21, 0.1, 0.2);
    require(unequal_R(0, 1) > 0.0, "Shared wheel measurement correlation retained");

    // Check the EKF linearization against a finite-difference derivative.
    af::State input; input << 0.3, -0.1, 0.4, 0.2, 0.5;
    const auto analytic = af::motionJacobian(input, 0.02);
    af::Covariance numeric;
    for (int i = 0; i < 5; ++i) {
      af::State plus = input, minus = input;
      plus(i) += 1e-6; minus(i) -= 1e-6;
      numeric.col(i) = (af::motionModel(plus, 0.02) - af::motionModel(minus, 0.02)) / 2e-6;
    }
    require((analytic - numeric).cwiseAbs().maxCoeff() < 1e-8, "Motion Jacobian derivative");

    af::PlanarEKF stationary;
    stationary.reset(af::Twist2::Zero(), R);
    for (int i = 0; i < 2000; ++i) {
      stationary.predict(0.01);
      stationary.updateWheel(af::Twist2::Zero(), R);
      stationary.updateGyro(0.0, 0.0004);
      if (i % 100 == 0) { covarianceValid(stationary); }
    }
    require(stationary.state().norm() < 1e-12, "Stationary input remains stationary");

    af::PlanarEKF circular;
    const af::Twist2 measured(0.1, 0.3);
    circular.reset(measured, R);
    for (int i = 0; i < 1000; ++i) {
      circular.predict(0.01);
      circular.updateWheel(measured, R);
      circular.updateGyro(0.3, 0.0004);
    }
    require(std::abs(circular.state()(0) - (0.1 / 0.3) * std::sin(3.0)) < 1e-6,
      "Circular path x agrees with analytic solution");
    require(std::abs(circular.state()(1) - (0.1 / 0.3) * (1.0 - std::cos(3.0))) < 1e-6,
      "Circular path y agrees with analytic solution");
    covarianceValid(circular);

    af::PlanarEKF wrap;
    wrap.reset(af::Twist2(0.0, 2.0), R);
    wrap.predict(4.0);
    require(std::abs(wrap.state()(2) - af::wrapAngle(8.0)) < 1e-10, "Yaw wraps correctly");
    const auto before = wrap.state();
    wrap.predict(0.0);
    require(wrap.state().isApprox(before), "Pause/zero dt leaves state unchanged");

    // A labelled synthetic case, NOT a claim about the user's simulation.
    // Biased wheel yaw rate 0.5; true rate and low-noise gyro 0.3 rad/s.
    af::TwistCovariance biased_R = af::TwistCovariance::Zero();
    biased_R(0, 0) = 0.001; biased_R(1, 1) = 0.04;
    af::PlanarEKF biased;
    biased.reset(af::Twist2(0.0, 0.5), biased_R);
    biased.updateGyro(0.3, 0.0001);
    for (int i = 0; i < 1000; ++i) {
      biased.predict(0.01);
      biased.updateWheel(af::Twist2(0.0, 0.5), biased_R);
      biased.updateGyro(0.3, 0.0001);
    }
    require(std::abs(af::wrapAngle(biased.state()(2) - 3.0)) < 0.02,
      "Reliable gyro limits yaw error in labelled synthetic wheel-bias case");
    covarianceValid(biased);

    bool rejected = false;
    try { biased.updateGyro(0.0, 0.0); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Zero/unknown variance cannot be used as perfect certainty");
    rejected = false;
    try { biased.predict(-0.01); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Backward dt rejected by mathematical core");
    rejected = false;
    try { biased.updateGyro(std::numeric_limits<double>::quiet_NaN(), 0.01); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Non-finite input rejected");
    std::cout << "PASS: wheel transform, correlated uncertainty, Jacobian, stationary and circular motion, "
              << "PSD covariance, yaw wrap, pause, synthetic gyro correction, invalid inputs.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n'; return 1;
  }
}
