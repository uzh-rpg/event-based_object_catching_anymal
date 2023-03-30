// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <array>
#include <sstream>
#include <string>

#include <ze/common/file_utils.hpp>
#include <ze/common/logging.hpp>
#include <ze/common/running_statistics.hpp>
#include <ze/common/string_utils.hpp>
#include <ze/common/types.hpp>

namespace ze {

/*! Collect samples and iteratively compute statistics.
 *
 * Usage:
\code{.cpp}
  DECLARE_STATISTICS(StatisticsName, stats, foo, bar);
  ze::StatisticsCollection stats;
  stats[StatisticsName::foo].addSample(12);
  stats[StatisticsName::foo].addSample(10);
  real_t variance = stats[StatisticsName::foo].var();
\endcode
*/
template <typename StatisticsEnum> class StatisticsCollection {
public:
  using Collection =
      std::array<RunningStatistics,
                 static_cast<uint32_t>(StatisticsEnum::dimension)>;
  using CollectionNames = std::vector<std::string>;

  StatisticsCollection() = delete;

  //! This constructor is used by the macro DECLARE_TIMER() below.
  StatisticsCollection(const std::string &statistics_names_comma_separated)
      : names_(splitString(statistics_names_comma_separated, ',')) {
    CHECK_EQ(names_.size(), collection_.size());
  }

  StatisticsCollection(const std::vector<std::string> &timer_names)
      : names_(timer_names) {
    CHECK_EQ(names_.size(), collection_.size());
  }

  ~StatisticsCollection() = default;

  inline RunningStatistics &operator[](StatisticsEnum s) {
    return collection_[static_cast<uint32_t>(s)];
  }

  inline const RunningStatistics &operator[](StatisticsEnum s) const {
    return collection_[static_cast<uint32_t>(s)];
  }

  constexpr size_t size() const noexcept { return collection_.size(); }

  //! Saves statistics to file in YAML format.
  void saveToFile(const std::string &directory, const std::string &filename) {
    std::ofstream fs;
    CHECK(isDir(directory));
    openOutputFileStream(joinPath(directory, filename), &fs);
    fs << *this;
  }

  inline const Collection &collection() const { return collection_; }

  inline const CollectionNames &names() const { return names_; }

private:
  Collection collection_;
  CollectionNames names_;
};

//! Print statistics collection.
//! Print Timer Collection:
template <typename StatisticsEnum>
std::ostream &
operator<<(std::ostream &out,
           const StatisticsCollection<StatisticsEnum> &statistics) {
  for (size_t i = 0u; i < statistics.size(); ++i) {
    out << statistics.names().at(i) << ":\n" << statistics.collection().at(i);
  }
  return out;
}

#define DECLARE_STATISTICS(classname, membername, ...)                         \
  enum class classname : uint32_t { __VA_ARGS__, dimension };                  \
  ze::StatisticsCollection<classname> membername { #__VA_ARGS__ }

} // end namespace ze
