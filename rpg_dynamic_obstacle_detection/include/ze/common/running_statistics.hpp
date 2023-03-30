// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <algorithm>
#include <ze/common/types.hpp>

namespace ze {

//! Collects samples and incrementally computes statistical properties.
//! http://www.johndcook.com/blog/standard_deviation/
class RunningStatistics {
public:
  RunningStatistics() = default;
  ~RunningStatistics() = default;

  inline void addSample(real_t x) {
    ++n_;
    min_ = std::min(min_, x);
    max_ = std::max(max_, x);
    sum_ += x;

    // Online variance computation [Knuth TAOCP vol 2, 3rd edition, page 232].
    if (n_ == 1u) {
      M_ = x;
      S_ = 0.0;
    } else {
      real_t M_new = M_ + (x - M_) / n_;
      S_ += (x - M_) * (x - M_new);
      M_ = M_new;
    }
  }

  inline real_t numSamples() const { return n_; }
  inline real_t min() const { return min_; }
  inline real_t max() const { return max_; }
  inline real_t sum() const { return sum_; }
  inline real_t mean() const { return M_; }

  // The use of (n-1) is due to
  // https://en.wikipedia.org/wiki/Bessel's_correction that's why the result for
  // small sample sizes in unit tests may not coincide with what you may expect.
  inline real_t var() const { return (n_ > 0u) ? S_ / (n_ - 1u) : 0.0; }
  inline real_t std() const { return std::sqrt(var()); }

  inline void reset() {
    n_ = 0;
    min_ = std::numeric_limits<real_t>::max();
    max_ = 0.0;
    sum_ = 0.0;
    M_ = 0.0;
    S_ = 0.0;
  }

private:
  uint32_t n_ = 0u;
  real_t min_ = std::numeric_limits<real_t>::max();
  real_t max_ = 0.0;
  real_t sum_ = 0.0;
  real_t M_ = 0.0;
  real_t S_ = 0.0;
};

//! Print statistics:
inline std::ostream &operator<<(std::ostream &out,
                                const RunningStatistics &stat) {
  out << "  num_samples: " << stat.numSamples() << "\n"
      << "  min: " << stat.min() << "\n"
      << "  max: " << stat.max() << "\n"
      << "  sum: " << stat.sum() << "\n"
      << "  mean: " << stat.mean() << "\n"
      << "  variance: " << stat.var() << "\n"
      << "  standard_deviation: " << stat.std() << "\n";
  return out;
}

} // end namespace ze
