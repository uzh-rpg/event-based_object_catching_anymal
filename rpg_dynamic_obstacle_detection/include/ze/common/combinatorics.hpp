// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <algorithm>
#include <functional>
#include <type_traits>
#include <vector>

#include <ze/common/types.hpp>

//! @file combinatorics.hpp
//! Some useful combinatorial functions. E.g. find matching index-entries in
//! two vectors.

namespace ze {

// -----------------------------------------------------------------------------
//! Returns indices (1. A, 2. B) of matching values in provided vectors.
template <typename T>
std::vector<std::pair<uint32_t, uint32_t>>
getMatchIndices(const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>> &A,
                const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>> &B,
                const std::function<bool(T)> &isValidId) {
  // First, create a sorted vector of the reference ids with corresponding
  // index.
  std::vector<std::pair<T, uint32_t>> B_indexed;
  B_indexed.reserve(B.size());
  for (int32_t i = 0; i < B.size(); ++i) {
    if (isValidId(B(i))) {
      B_indexed.push_back(std::make_pair(B(i), i));
    }
  }
  std::sort(
      B_indexed.begin(), B_indexed.end(),
      [](const std::pair<T, uint32_t> &lhs, const std::pair<T, uint32_t> &rhs) {
        return lhs.first < rhs.first;
      });

  // For each current id, find matching reference id.
  std::vector<std::pair<uint32_t, uint32_t>> matches_AB;
  matches_AB.reserve(B_indexed.size());
  for (int32_t i = 0; i < A.size(); ++i) {
    if (isValidId(A(i))) {
      // Efficient search for matching id in sorted range.
      auto it = std::lower_bound(B_indexed.begin(), B_indexed.end(), A(i),
                                 [](const std::pair<T, uint32_t> &lhs, T rhs) {
                                   return lhs.first < rhs;
                                 });
      if (it != B_indexed.end() && it->first == A(i)) {
        // Success.
        matches_AB.push_back(std::make_pair(i, it->second));
      }
    }
  }
  return matches_AB;
}

// -----------------------------------------------------------------------------
//! Returns indices of A that don't have a matching index in B.
template <typename T>
std::vector<uint32_t> getUnmatchedIndices(
    const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>> &A,
    const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, 1>> &B,
    const std::function<bool(T)> &isValidId) {
  // First, create a sorted vector of the reference ids with corresponding
  // index.
  std::vector<std::pair<T, uint32_t>> B_indexed;
  B_indexed.reserve(B.size());
  for (int32_t i = 0; i < B.size(); ++i) {
    if (isValidId(B(i))) {
      B_indexed.push_back(std::make_pair(B(i), i));
    }
  }
  std::sort(
      B_indexed.begin(), B_indexed.end(),
      [](const std::pair<T, uint32_t> &lhs, const std::pair<T, uint32_t> &rhs) {
        return lhs.first < rhs.first;
      });

  // For each current id, find matching reference id.
  std::vector<uint32_t> unmached_A;
  unmached_A.reserve(B_indexed.size());
  for (int32_t i = 0; i < A.size(); ++i) {
    if (isValidId(A(i))) {
      // Efficient search for matching id in sorted range.
      auto it = std::lower_bound(B_indexed.begin(), B_indexed.end(), A(i),
                                 [](const std::pair<T, uint32_t> &lhs, T rhs) {
                                   return lhs.first < rhs;
                                 });
      if (it == B_indexed.end() || it->first != A(i)) {
        // No match found:
        unmached_A.push_back(i);
      }
    }
  }
  return unmached_A;
}

// -----------------------------------------------------------------------------
template <typename T>
std::vector<T> getOutlierIndicesFromInlierIndices(std::vector<T> &inliers,
                                                  const T size) {
  static_assert(std::is_integral<T>::value, "Type T must be an integral type");

  std::sort(inliers.begin(), inliers.end(), std::less<T>());
  std::vector<T> outliers;
  outliers.reserve(size - inliers.size());

  T k = 0;
  for (T i = 0; i < size; ++i) {
    if (k < static_cast<T>(inliers.size()) && inliers[k] < i) {
      ++k;
    }

    if (k >= size || inliers[k] != i) {
      outliers.push_back(i);
    }
  }

  return outliers;
}

} // namespace ze
