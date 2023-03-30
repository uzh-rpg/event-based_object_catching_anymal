// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/types.hpp>

//! @file math.hpp
//! Common math utilities.

namespace ze {

template <typename Scalar> constexpr Scalar radToDeg(Scalar rad) {
  return rad * 180.0 / M_PI;
}

template <typename Scalar> constexpr Scalar degToRad(Scalar deg) {
  return deg * M_PI / 180.0;
}

} // namespace ze
