// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/macros.hpp>
#include <ze/common/random.hpp>
#include <ze/common/types.hpp>

//! @file random_matrix.hpp
//! Sample matrices and vectors from uniform or normal distributions.

namespace ze {

//------------------------------------------------------------------------------
//! A sampler for uncorrelated noise vectors.
template <size_t DIM> class RandomVectorSampler {
public:
  ZE_POINTER_TYPEDEFS(RandomVectorSampler);

  typedef Eigen::Matrix<real_t, DIM, DIM> covariance_matrix_t;
  typedef Eigen::Matrix<real_t, DIM, 1> covariance_vector_t;
  typedef Eigen::Matrix<real_t, DIM, 1> sigma_vector_t;
  typedef Eigen::Matrix<real_t, DIM, 1> noise_vector_t;

  //! Get a noise sample.
  noise_vector_t sample() {
    noise_vector_t noise;
    for (size_t i = 0; i < DIM; ++i) {
      // The gaussian takes a standard deviation as input.
      noise(i) =
          sampleNormalDistribution<real_t>(deterministic_, 0.0, sigma_(i));
    }
    return noise;
  }

  static Ptr sigmas(const sigma_vector_t &sigmas, bool deterministic = false) {
    Ptr noise(new RandomVectorSampler(deterministic));
    noise->sigma_ = sigmas;
    return noise;
  }

  static Ptr variances(const covariance_vector_t &variances,
                       bool deterministic = false) {
    Ptr noise(new RandomVectorSampler(deterministic));
    noise->sigma_ = variances.cwiseSqrt();
    return noise;
  }

protected:
  RandomVectorSampler(bool deteterministic) : deterministic_(deteterministic) {}

private:
  const bool deterministic_;
  sigma_vector_t sigma_;
};

//------------------------------------------------------------------------------
inline MatrixX randomMatrixUniformDistributed(int rows, int cols,
                                              bool deterministic = false,
                                              real_t from = 0.0,
                                              real_t to = 1.0) {
  DEBUG_CHECK_GT(rows, 0);
  DEBUG_CHECK_GT(cols, 0);
  MatrixX m(rows, cols);
  for (int x = 0; x < cols; ++x) {
    for (int y = 0; y < rows; ++y) {
      m(y, x) = sampleUniformRealDistribution(deterministic, from, to);
    }
  }
  return m;
}

template <int rows, int cols>
Eigen::Matrix<real_t, rows, cols>
randomMatrixUniformDistributed(bool deterministic = false, real_t from = 0.0,
                               real_t to = 1.0) {
  return randomMatrixUniformDistributed(rows, cols, deterministic, from, to);
}

template <int size>
Eigen::Matrix<real_t, size, 1>
randomVectorUniformDistributed(bool deterministic = false, real_t from = 0.0,
                               real_t to = 1.0) {
  return randomMatrixUniformDistributed<size, 1>(deterministic, from, to);
}

//------------------------------------------------------------------------------
inline MatrixX randomMatrixNormalDistributed(int rows, int cols,
                                             bool deterministic = false,
                                             real_t mean = 0.0,
                                             real_t sigma = 1.0) {
  DEBUG_CHECK_GT(rows, 0);
  DEBUG_CHECK_GT(cols, 0);
  MatrixX m(rows, cols);
  for (int x = 0; x < cols; ++x) {
    for (int y = 0; y < rows; ++y) {
      m(y, x) = sampleNormalDistribution(deterministic, mean, sigma);
    }
  }
  return m;
}

template <int rows, int cols>
Eigen::Matrix<real_t, rows, cols>
randomMatrixNormalDistributed(bool deterministic = false, real_t mean = 0.0,
                              real_t sigma = 1.0) {
  return randomMatrixNormalDistributed(rows, cols, deterministic, mean, sigma);
}

template <int size>
Eigen::Matrix<real_t, size, 1>
randomVectorNormalDistributed(bool deterministic = false, real_t mean = 0.0,
                              real_t sigma = 1.0) {
  return randomMatrixNormalDistributed<size, 1>(deterministic, mean, sigma);
}

} // namespace ze
