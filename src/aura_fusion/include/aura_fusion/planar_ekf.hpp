#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aura_fusion
{
using State = Eigen::Matrix<double, 5, 1>;
using Covariance = Eigen::Matrix<double, 5, 5>;
using Twist2 = Eigen::Vector2d;
using TwistCovariance = Eigen::Matrix2d;

inline double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline Twist2 wheelTwist(double left, double right, double radius, double separation)
{
  return Twist2(radius * (left + right) / 2.0, radius * (right - left) / separation);
}

// Transform independent left/right angular-speed uncertainty into [v, w].
// If the wheel variances differ, v and w are correlated; retain that term.
inline TwistCovariance wheelCovariance(
  double radius, double separation, double left_std, double right_std)
{
  Eigen::Matrix2d A;
  A << radius / 2.0, radius / 2.0, -radius / separation, radius / separation;
  Eigen::Matrix2d wheel_R = Eigen::Matrix2d::Zero();
  wheel_R(0, 0) = left_std * left_std;
  wheel_R(1, 1) = right_std * right_std;
  return A * wheel_R * A.transpose();
}

// State order: [x metres, y metres, heading radians, v m/s, w rad/s].
inline State motionModel(const State& state, double dt)
{
  State next = state;
  const double middle = state(2) + 0.5 * state(4) * dt;
  next(0) += state(3) * dt * std::cos(middle);
  next(1) += state(3) * dt * std::sin(middle);
  next(2) = wrapAngle(state(2) + state(4) * dt);
  return next;
}

// F is the derivative of the motion model with respect to the old state.
inline Covariance motionJacobian(const State& state, double dt)
{
  const double middle = state(2) + 0.5 * state(4) * dt;
  const double c = std::cos(middle), s = std::sin(middle), v = state(3);
  Covariance F = Covariance::Identity();
  F(0, 2) = -v * dt * s;
  F(0, 3) = dt * c;
  F(0, 4) = -0.5 * v * dt * dt * s;
  F(1, 2) = v * dt * c;
  F(1, 3) = dt * s;
  F(1, 4) = 0.5 * v * dt * dt * c;
  F(2, 4) = dt;
  return F;
}

struct ModelNoise
{
  // Continuous variance growth rates, not per-message standard deviations.
  double speed_variance_per_second = 0.04;     // (m/s)^2 / s
  double turn_variance_per_second = 0.25;     // (rad/s)^2 / s
  double position_variance_per_second = 1e-6; // m^2 / s, unmodelled motion
};

class PlanarEKF
{
public:
  explicit PlanarEKF(ModelNoise noise = {}) : noise_(noise)
  {
    if (!std::isfinite(noise.speed_variance_per_second) ||
        !std::isfinite(noise.turn_variance_per_second) ||
        !std::isfinite(noise.position_variance_per_second) ||
        noise.speed_variance_per_second < 0.0 || noise.turn_variance_per_second < 0.0 ||
        noise.position_variance_per_second < 0.0) {
      throw std::invalid_argument("Invalid process noise");
    }
  }

  void reset(const Twist2& wheel, const TwistCovariance& wheel_R)
  {
    validateMeasurement<2>(wheel, wheel_R);
    state_.setZero();
    state_.tail<2>() = wheel;
    covariance_.setZero();
    // The origin is defined to be the starting pose; this is relative odometry.
    covariance_.bottomRightCorner<2, 2>() = wheel_R;
  }

  void predict(double dt)
  {
    if (!std::isfinite(dt) || dt < 0.0) {
      throw std::invalid_argument("Prediction dt must be finite and nonnegative");
    }
    // Short steps keep the midpoint approximation accurate on a larger gap.
    while (dt > 1e-12) {
      const double step = std::min(dt, 0.02);
      const Covariance F = motionJacobian(state_, step);
      state_ = motionModel(state_, step);
      Covariance Q = Covariance::Zero();
      Q(0, 0) = Q(1, 1) = noise_.position_variance_per_second * step;
      Q(3, 3) = noise_.speed_variance_per_second * step;
      Q(4, 4) = noise_.turn_variance_per_second * step;
      covariance_ = (F * covariance_ * F.transpose() + Q).eval();
      covariance_ = (0.5 * (covariance_ + covariance_.transpose())).eval();
      dt -= step;
    }
  }

  void updateWheel(const Twist2& measured, const TwistCovariance& R)
  {
    Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
    H(0, 3) = 1.0; H(1, 4) = 1.0;
    update<2>(measured, H, R);
  }

  void updateGyro(double measured_w, double variance)
  {
    Eigen::Matrix<double, 1, 1> z, R;
    z << measured_w; R << variance;
    Eigen::Matrix<double, 1, 5> H = Eigen::Matrix<double, 1, 5>::Zero();
    H(0, 4) = 1.0;
    update<1>(z, H, R);
  }

  const State& state() const { return state_; }
  const Covariance& covariance() const { return covariance_; }

private:
  template<int M>
  static void validateMeasurement(
    const Eigen::Matrix<double, M, 1>& z, const Eigen::Matrix<double, M, M>& R)
  {
    if (!z.allFinite() || !R.allFinite() || !R.isApprox(R.transpose(), 1e-10)) {
      throw std::invalid_argument("Invalid measurement/covariance");
    }
    Eigen::LDLT<Eigen::Matrix<double, M, M>> ldlt(R);
    if (ldlt.info() != Eigen::Success || ldlt.vectorD().minCoeff() <= 0.0) {
      throw std::invalid_argument("Measurement covariance must be positive definite");
    }
  }

  template<int M>
  void update(
    const Eigen::Matrix<double, M, 1>& z, const Eigen::Matrix<double, M, 5>& H,
    const Eigen::Matrix<double, M, M>& R)
  {
    validateMeasurement<M>(z, R);
    const Eigen::Matrix<double, M, 1> innovation = z - H * state_;
    const Eigen::Matrix<double, M, M> S = H * covariance_ * H.transpose() + R;
    const Eigen::Matrix<double, 5, M> PHt = covariance_ * H.transpose();
    Eigen::LDLT<Eigen::Matrix<double, M, M>> solve(S);
    if (solve.info() != Eigen::Success || solve.vectorD().minCoeff() <= 0.0) {
      throw std::runtime_error("Innovation covariance is not positive definite");
    }
    // Solve a linear system instead of explicitly inverting S.
    const Eigen::Matrix<double, 5, M> K = solve.solve(PHt.transpose()).transpose();
    state_ += K * innovation;
    state_(2) = wrapAngle(state_(2));
    // Joseph form preserves covariance symmetry/positive semidefiniteness better.
    const Covariance I_KH = Covariance::Identity() - K * H;
    covariance_ = (I_KH * covariance_ * I_KH.transpose() + K * R * K.transpose()).eval();
    covariance_ = (0.5 * (covariance_ + covariance_.transpose())).eval();
    if (!state_.allFinite() || !covariance_.allFinite()) {
      throw std::runtime_error("Non-finite filter result");
    }
  }

  ModelNoise noise_;
  State state_ = State::Zero();
  Covariance covariance_ = Covariance::Zero();
};
}  // namespace aura_fusion
