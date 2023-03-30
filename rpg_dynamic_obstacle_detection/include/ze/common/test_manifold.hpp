// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/logging.hpp>

#include <ze/common/manifold.hpp>
#include <ze/common/numerical_derivative.hpp>

namespace ze {

template <typename T>
void testManifoldInvariants(const T &a, const T &b, real_t tol = 1e-9) {
  CHECK(traits<T>::equals(a, a));
  typename traits<T>::TangentVector v = traits<T>::local(a, b);
  T c = traits<T>::retract(a, v);
  CHECK(traits<T>::equals(b, c, tol));
}

template <typename T>
void testRetractJacobians(const T &a, const T &b, real_t tol = 1e-9) {
  using namespace std::placeholders; // for _1
  typename traits<T>::Jacobian H1, H2;
  typename traits<T>::TangentVector v = traits<T>::local(a, b);
  T c = traits<T>::retract(a, v, &H1, &H2);
  typename traits<T>::Jacobian H1_numerical = numericalDerivative<T, T>(
      std::bind(traits<T>::retract, _1, v, nullptr, nullptr), a);
  CHECK(traits<typename traits<T>::Jacobian>::equals(H1, H1_numerical, tol));

  typename traits<T>::Jacobian H2_numerical =
      numericalDerivative<T, typename traits<T>::TangentVector>(
          std::bind(traits<T>::retract, a, _1, nullptr, nullptr), v);
  CHECK(traits<typename traits<T>::Jacobian>::equals(H2, H2_numerical, tol));
}

template <typename T>
void testLocalJacobians(const T &a, const T &b, real_t tol = 1e-9) {
  using namespace std::placeholders; // for _1
  typename traits<T>::Jacobian H1, H2;
  typename traits<T>::TangentVector v = traits<T>::local(a, b, &H1, &H2);
  typename traits<T>::Jacobian H1_numerical =
      numericalDerivative<typename traits<T>::TangentVector, T>(
          std::bind(traits<T>::local, _1, b, nullptr, nullptr), a);
  CHECK(traits<typename traits<T>::Jacobian>::equals(H1, H1_numerical, tol));

  typename traits<T>::Jacobian H2_numerical =
      numericalDerivative<typename traits<T>::TangentVector, T>(
          std::bind(traits<T>::local, a, _1, nullptr, nullptr), b);
  CHECK(traits<typename traits<T>::Jacobian>::equals(H2, H2_numerical, tol));
}
} // namespace ze
