// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <Eigen/Core>

#include <yaml-cpp/yaml.h>
#include <ze/common/logging.hpp>

namespace YAML {

template <typename T>
T extractChild(const YAML::Node &node, const std::string &key) {
  if (!node.IsMap()) {
    throw YAML::Exception(node.Mark(), "Node is not a map");
  }

  const YAML::Node child = node[key];
  if (!child) {
    throw YAML::Exception(node.Mark(), "key '" + key + "' does not exist");
  }

  return child.as<T>();
}

/**
 * yaml serialization helper function for the Eigen3 Matrix object.
 * The matrix is a base class for dense matrices.
 * http://eigen.tuxfamily.org/dox-devel/TutorialMatrixClass.html
 *
 * see yaml-cpp for how the extraction of complex types works
 *
 * Inspired by ETHZ-ASL ASLAM_CV_COMMON
 */
template <class Scalar, int A, int B, int C, int D, int E>
struct convert<Eigen::Matrix<Scalar, A, B, C, D, E>> {
  static bool decode(const Node &node,
                     Eigen::Matrix<Scalar, A, B, C, D, E> &M) {
    if (!node.IsMap()) {
      LOG(ERROR) << "Unable to parse the matrix because the node is not a map.";
      return false;
    }

    typedef typename Eigen::Matrix<Scalar, A, B, C, D, E>::Index IndexType;
    IndexType rows = node["rows"].as<IndexType>();
    IndexType cols = node["cols"].as<IndexType>();

    if ((A != Eigen::Dynamic && rows != A) ||
        (B != Eigen::Dynamic && cols != B)) {
      LOG(ERROR) << "The matrix is the wrong size (rows, cols). Wanted: ("
                 << (A == Eigen::Dynamic ? rows : A) << ","
                 << (B == Eigen::Dynamic ? cols : B) << "), got (" << rows
                 << ", " << cols << ")";
      return false;
    }

    M.resize(rows, cols);

    size_t expected_size = M.rows() * M.cols();
    if (!node["data"].IsSequence()) {
      LOG(ERROR) << "The matrix data is not a sequence.";
      return false;
    }
    if (node["data"].size() != expected_size) {
      LOG(ERROR) << "The data sequence is the wrong size. Wanted: "
                 << expected_size << ", got: " << node["data"].size();
      return false;
    }

    YAML::const_iterator it = node["data"].begin();
    YAML::const_iterator it_end = node["data"].end();
    if (rows > 0 && cols > 0) {
      for (IndexType i = 0; i < rows; ++i) {
        for (IndexType j = 0; j < cols; ++j) {
          CHECK(it != it_end);
          M(i, j) = it->as<Scalar>();
          ++it;
        }
      }
    }
    return true;
  }
};

} // namespace YAML
