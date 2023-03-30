// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include <ze/common/logging.hpp>

//! @file string_utilties.hpp
//! Various utilities to work with std::string.

namespace ze {

inline std::string &leftTrimString(std::string &s) {
  s.erase(s.begin(),
          std::find_if(s.begin(), s.end(),
                       std::not1(std::ptr_fun<int, int>(std::isspace))));
  return s;
}

inline std::string &rightTrimString(std::string &s) {
  s.erase(std::find_if(s.rbegin(), s.rend(),
                       std::not1(std::ptr_fun<int, int>(std::isspace)))
              .base(),
          s.end());
  return s;
}

inline std::string &trimString(std::string &s) {
  return leftTrimString(rightTrimString(s));
}

inline std::string &ensureLeftSlash(std::string &s) {
  CHECK_GE(s.size(), 1u);
  if (s[0] != '/') {
    s.insert(0, "/");
  }
  return s;
}

inline std::string ensureLeftSlash(const std::string &s) {
  CHECK_GE(s.size(), 1u);
  std::string s_copy = s;
  if (s_copy[0] != '/') {
    s_copy.insert(0, "/");
  }
  return s_copy;
}

inline std::string ensureNoLeftSlash(const std::string &s) {
  CHECK_GE(s.size(), 1u);
  if (s[0] != '/') {
    return s;
  }
  std::string s_copy = s;
  s_copy.erase(0, 1);
  CHECK(s_copy[0] != '/');
  return s_copy;
}

inline std::string ensureRightSlash(const std::string &s) {
  if (s.size() == 0) {
    return "/";
  }
  if (s[s.size() - 1] == '/') {
    return s;
  }
  return s + "/";
}

inline std::vector<std::string> splitString(const std::string &s, char delim) {
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> items;
  while (std::getline(ss, item, delim)) {
    items.push_back(trimString(item));
  }
  return items;
}

inline std::vector<int> readIntsFromString(const std::string &s,
                                           char delim = ',') {
  std::vector<std::string> items = splitString(s, delim);
  std::vector<int> values;
  for (const std::string s : items) {
    values.push_back(stoi(s));
  }
  return values;
}

inline bool replaceInString(std::string &str, const std::string &from,
                            const std::string &to) {
  size_t start_pos = str.find(from);
  if (start_pos == std::string::npos) {
    return false;
  }
  str.replace(start_pos, from.length(), to);
  return true;
}

} // namespace ze
