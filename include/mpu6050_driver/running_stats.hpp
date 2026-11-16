// Copyright (c) 2026, mpu6050_driver contributors. BSD-3-Clause (see LICENSE).
//
// Mean and standard deviation of 3-axis samples (Welford's algorithm, which
// stays numerically stable for thousands of samples). Used for calibration.

#ifndef MPU6050_DRIVER__RUNNING_STATS_HPP_
#define MPU6050_DRIVER__RUNNING_STATS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace mpu6050_driver
{

class RunningStats3
{
public:
  void reset()
  {
    n_ = 0;
    mean_ = {};
    m2_ = {};
  }

  void add(const std::array<double, 3> & x)
  {
    ++n_;
    for (std::size_t i = 0; i < 3; ++i) {
      const double delta = x[i] - mean_[i];
      mean_[i] += delta / static_cast<double>(n_);
      m2_[i] += delta * (x[i] - mean_[i]);
    }
  }

  std::size_t count() const {return n_;}

  std::array<double, 3> mean() const {return mean_;}

  /// Sample standard deviation per axis (0 with fewer than 2 samples).
  std::array<double, 3> stddev() const
  {
    std::array<double, 3> s{};
    if (n_ > 1) {
      for (std::size_t i = 0; i < 3; ++i) {
        s[i] = std::sqrt(m2_[i] / static_cast<double>(n_ - 1));
      }
    }
    return s;
  }

  /// Largest per-axis standard deviation.
  double maxStddev() const
  {
    const auto s = stddev();
    return *std::max_element(s.begin(), s.end());
  }

private:
  std::size_t n_{0};
  std::array<double, 3> mean_{};
  std::array<double, 3> m2_{};
};

}  // namespace mpu6050_driver

#endif  // MPU6050_DRIVER__RUNNING_STATS_HPP_
