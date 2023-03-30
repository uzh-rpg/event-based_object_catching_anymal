// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/types.hpp>

namespace ze {

//! Utilities for working with timestamps.
//!
//! Important: Always store int64_t nanosecond timestamps! We use signed type
//!            to avoid errors when taking differences and we use nanoseconds
//!            when saving to file to have a unique type for lookups in
//!            dictionaries/maps.

//! Seconds to nanoseconds.
inline constexpr int64_t secToNanosec(real_t seconds) {
  return static_cast<int64_t>(seconds * 1e9);
}

//! Milliseconds to nanoseconds.
inline constexpr int64_t millisecToNanosec(real_t milliseconds) {
  return static_cast<int64_t>(milliseconds * 1e6);
}

//! Nanoseconds to seconds.
//! WARNING: Don't pass very large or small numbers to this function as the
//!          representability of the float value does not capture nanoseconds
//!          resolution. The resulting accuracy will be in the order of
//!          hundreds of nanoseconds.
inline constexpr real_t nanosecToSecTrunc(int64_t nanoseconds) {
  return static_cast<real_t>(nanoseconds) / 1e9;
}

//! Nanoseconds to milliseconds.
//! WARNING: Don't pass very large or very small numbers to this function as the
//!          representability of the float value does not capture nanoseconds
//!          resolution.
inline constexpr real_t nanosecToMillisecTrunc(int64_t nanoseconds) {
  return static_cast<real_t>(nanoseconds) / 1e6;
}

//! Return total nanoseconds from seconds and nanoseconds pair.
inline constexpr int64_t nanosecFromSecAndNanosec(int32_t sec, int32_t nsec) {
  return static_cast<int64_t>(sec) * 1000000000ll + static_cast<int64_t>(nsec);
}

} // namespace ze
