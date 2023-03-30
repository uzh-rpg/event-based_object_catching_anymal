// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/running_statistics.hpp>
#include <ze/common/timer.hpp>
#include <ze/common/types.hpp>

namespace ze {

// fwd
class TimedScope;

//! Collect statistics over multiple timings in milliseconds.
class TimerStatistics {
public:
  inline void start() { t_.start(); }

  //! Using the concept of "Initialization is Resource Acquisition" idiom, this
  //! function returns a timer object. When this timer object is destructed,
  //! the timer is stopped.
  inline TimedScope timeScope();

  inline real_t stop() {
    real_t t = t_.stopAndGetMilliseconds();
    stat_.addSample(t);
    return t;
  }

  inline real_t numTimings() const { return stat_.numSamples(); }
  inline real_t accumulated() const { return stat_.sum(); }
  inline real_t min() const { return stat_.min(); }
  inline real_t max() const { return stat_.max(); }
  inline real_t mean() const { return stat_.mean(); }
  inline real_t variance() const { return stat_.var(); }
  inline real_t standarDeviation() const { return stat_.std(); }
  inline void reset() { stat_.reset(); }
  inline const RunningStatistics &statistics() const { return stat_; }

private:
  Timer t_;
  RunningStatistics stat_;
};

//! This object is return from TimerStatistics::timeScope()
class TimedScope {
public:
  TimedScope() = delete;

  TimedScope(TimerStatistics *timer) : timer_(timer) { timer_->start(); }

  ~TimedScope() { timer_->stop(); }

private:
  TimerStatistics *timer_;
};

inline TimedScope TimerStatistics::timeScope() {
  // Returning a pointer to this should be safe, as inserting more elements in
  // the unordered map does not invalidate any references to other elements in
  // the unordered map (http://stackoverflow.com/questions/16781886).
  return TimedScope(this);
}

} // end namespace ze
