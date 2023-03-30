// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <chrono>

#include <ze/common/time_conversions.hpp>
#include <ze/common/types.hpp>

namespace ze {

//! Simple timing utilty.
class Timer {
public:
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using ns = std::chrono::nanoseconds;
  using ms = std::chrono::milliseconds;

  //! The constructor directly starts the timer.
  Timer() : start_time_(Clock::now()) {}

  inline void start() { start_time_ = Clock::now(); }

  inline int64_t stopAndGetNanoseconds() {
    const TimePoint end_time(Clock::now());
    ns duration = std::chrono::duration_cast<ns>(end_time - start_time_);
    return duration.count();
  }

  inline real_t stopAndGetMilliseconds() {
    return nanosecToMillisecTrunc(stopAndGetNanoseconds());
  }

  inline real_t stopAndGetSeconds() {
    return nanosecToSecTrunc(stopAndGetNanoseconds());
  }

private:
  TimePoint start_time_;
};

} // end namespace ze
