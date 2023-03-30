// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <functional>
#include <iostream>
#include <ze/common/manifold.hpp>
#include <ze/common/types.hpp>

namespace ze {

//! Template function to compute numerical derivatives. See unit tests for
//! examples. The traits used for this functions are defined in
//! common/manifold.h
template <class Y, class X>
typename Eigen::Matrix<real_t, traits<Y>::dimension, traits<X>::dimension>
numericalDerivative(std::function<Y(const X &)> h, const X &x,
                    real_t delta = 1e-5) {
  typedef
      typename Eigen::Matrix<real_t, traits<Y>::dimension, traits<X>::dimension>
          Jacobian;
  typedef typename traits<Y>::TangentVector TangentY;
  typedef typename traits<X>::TangentVector TangentX;

  const int N_X = traits<X>::getDimension(x);

  // Get value at x.
  Y hx = h(x);

  const int N_Y = traits<Y>::getDimension(hx);

  // Prepare a tangent vector to perturb x.
  TangentX dx(N_X, 1);
  dx.setZero();

  // Compute numerical Jacobian column by column.
  Jacobian H(N_Y, N_X);
  H.setZero();

  real_t factor = 1.0 / (2.0 * delta);

  for (int i = 0; i < N_X; ++i) {
    dx(i) = delta;
    TangentY dy1 = traits<Y>::local(hx, h(traits<X>::retract(x, dx)));
    dx(i) = -delta;
    TangentY dy2 = traits<Y>::local(hx, h(traits<X>::retract(x, dx)));
    dx(i) = 0;
    H.col(i) << (dy1 - dy2) * factor;
  }
  return H;
}

} // namespace ze
