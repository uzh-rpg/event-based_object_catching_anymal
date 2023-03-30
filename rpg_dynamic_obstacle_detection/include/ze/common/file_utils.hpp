// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <fstream>
#include <iostream>
#include <ze/common/path_utils.hpp>
#include <ze/common/string_utils.hpp>

//! @file file_utils.hpp
//! Utility to open read or write filestreams and do the appropriate checks.

namespace ze {

inline void openFileStream(const std::string &filename, std::ifstream *fs) {
  CHECK_NOTNULL(fs);
  CHECK(fileExists(filename)) << "File does not exist: " << filename;
  fs->open(filename.c_str(), std::ios::in);
  CHECK(*fs);
  CHECK(fs->is_open()) << "Failed to open file: " << filename;
  CHECK(!fs->eof()) << "File seems to contain no content!";
}

inline void openFileStreamAndCheckHeader(const std::string &filename,
                                         const std::string &header,
                                         std::ifstream *fs) {
  openFileStream(filename, fs);
  std::string line;
  std::getline(*fs, line);
  CHECK_EQ(line, header) << "Invalid header.";
}

inline void openOutputFileStream(const std::string &filename,
                                 std::ofstream *fs) {
  CHECK_NOTNULL(fs);
  fs->open(filename.c_str(), std::ios::out);
  CHECK(*fs) << "Failed to open filestream " << filename;
  CHECK(fs->is_open()) << "Failed to open file: " << filename;
}

} // namespace ze
