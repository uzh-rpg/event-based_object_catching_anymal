// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <utility>
#include <vector>
#include <ze/common/logging.hpp>

#include <ze/common/stl_utils.hpp>

//! @file statistics.hpp
//! Various utilities, e.g. to compute statistical properties of vectors.

namespace ze {

//! Does not take const-ref because vector will be sorted.
template <typename Scalar>
std::pair<Scalar, bool> median(std::vector<Scalar> &v) {
  if (v.size() == 0) {
    LOG(WARNING) << "Median computation of empty vector.";
    return std::make_pair(Scalar{0}, false);
  }
  const size_t center = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + center, v.end());
  Scalar median = v[center];
  return std::make_pair(median, true);
}

template <typename DerivedVec>
std::pair<typename DerivedVec::Scalar, bool>
median(const Eigen::MatrixBase<DerivedVec> &v) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedVec);
  auto w = eigenVectorToStlVector(v);
  return median<typename DerivedVec::Scalar>(w);
}

template <class T> inline T normPdf(const T x, const T mean, const T sigma) {
  T exponent = x - mean;
  exponent *= -exponent;
  exponent /= 2 * sigma * sigma;
  T result = std::exp(exponent);
  result /= sigma * std::sqrt(2 * M_PI);
  return result;
}

//! Calculate the covariance of a matrix of measurements. The measurements are
//! stacked column-wise in the matrix.
inline const MatrixX measurementCovariance(const Eigen::Ref<MatrixX> &values) {
  MatrixX zero_mean = values.colwise() - values.rowwise().mean();

  return (zero_mean * zero_mean.adjoint()) / (zero_mean.cols() - 1);
}

} // namespace ze
