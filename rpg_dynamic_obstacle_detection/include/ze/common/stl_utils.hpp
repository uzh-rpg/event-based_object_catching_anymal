// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <numeric>
#include <vector>
#include <ze/common/logging.hpp>
#include <ze/common/types.hpp>

//! @file stl_utils.hpp
//! Various utilities to work with the standard template library.

namespace ze {

// -----------------------------------------------------------------------------
//! Transform Eigen::Vector to std::vector.
template <typename DerivedVec>
std::vector<typename DerivedVec::Scalar>
eigenVectorToStlVector(const Eigen::MatrixBase<DerivedVec> &v) {
  //! @todo: both data is continuous, can we do this more efficiently?
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedVec);
  std::vector<typename DerivedVec::Scalar> rv(v.size());
  for (int i = 0; i < v.size(); ++i) {
    rv[i] = v(i);
  }
  return rv;
}

// -----------------------------------------------------------------------------
//! @return Returns a vector of indices form start to stop.
inline std::vector<uint32_t> range(uint32_t start, uint32_t stop) {
  DEBUG_CHECK_GE(stop, start);
  std::vector<uint32_t> vec(stop - start);
  std::iota(vec.begin(), vec.end(), start);
  return vec;
}

// -----------------------------------------------------------------------------
//! @return Returns a vector of indices form 0 to stop.
inline std::vector<uint32_t> range(uint32_t stop) { return range(0u, stop); }

} // namespace ze
