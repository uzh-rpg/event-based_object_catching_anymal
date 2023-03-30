// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <array>
#include <sstream>
#include <string>

#include <ze/common/file_utils.hpp>
#include <ze/common/logging.hpp>
#include <ze/common/running_statistics.hpp>
#include <ze/common/string_utils.hpp>
#include <ze/common/time_conversions.hpp>
#include <ze/common/timer_statistics.hpp>
#include <ze/common/types.hpp>

namespace ze {

/*! Collect statistics over multiple timings in milliseconds.
 *
 * Usage with explicit start and stop:
\code{.cpp}
  // Use the macro to declare the timer:
  DECLARE_TIMER(TimerName, timers, foo, bar);
  timers[TimerName::foo].start();
  ...
  timers[TimerName::foo].stop();
\endcode
 * Or RAII-style, use a TimedScope object that stops the timer when it
 * goes out of scope.
\code{.cpp}
  {
    auto t = timers[TimerName::foo].timeScope();
    ...
  }
\endcode
*/
template <typename TimerEnum> class TimerCollection {
public:
  using Timers =
      std::array<TimerStatistics, static_cast<uint32_t>(TimerEnum::dimension)>;
  using TimerNames = std::vector<std::string>;

  TimerCollection() = delete;

  //! This constructor is used by the macro DECLARE_TIMER() below.
  TimerCollection(const std::string &timer_names_comma_separated)
      : names_(splitString(timer_names_comma_separated, ',')) {
    CHECK_EQ(names_.size(), timers_.size());
  }

  TimerCollection(const std::vector<std::string> &timer_names)
      : names_(timer_names) {
    CHECK_EQ(names_.size(), timers_.size());
  }

  ~TimerCollection() = default;

  inline TimerStatistics &operator[](TimerEnum t) {
    return timers_[static_cast<uint32_t>(t)];
  }

  inline const TimerStatistics &operator[](TimerEnum t) const {
    return timers_[static_cast<uint32_t>(t)];
  }

  constexpr size_t size() const noexcept { return timers_.size(); }

  //! Saves timings to file in YAML format.
  inline void saveToFile(const std::string &directory,
                         const std::string &filename) const {
    std::ofstream fs;
    CHECK(isDir(directory));
    openOutputFileStream(joinPath(directory, filename), &fs);
    fs << *this;
  }

  inline const Timers &timers() const { return timers_; }

  inline const TimerNames &names() const { return names_; }

private:
  Timers timers_;
  std::vector<std::string> names_;
};

//! Print Timer Collection:
template <typename TimerEnum>
std::ostream &operator<<(std::ostream &out,
                         const TimerCollection<TimerEnum> &timers) {
  for (size_t i = 0u; i < timers.size(); ++i) {
    out << timers.names().at(i) << ":\n" << timers.timers().at(i).statistics();
  }
  return out;
}

} // end namespace ze

#define DECLARE_TIMER(classname, membername, ...)                              \
  enum class classname : uint32_t { __VA_ARGS__, dimension };                  \
  ze::TimerCollection<classname> membername { #__VA_ARGS__ }
