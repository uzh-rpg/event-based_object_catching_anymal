// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <map>
#include <mutex>
#include <thread>
#include <tuple>
#include <utility>

#include <ze/common/logging.hpp>
#include <ze/common/time_conversions.hpp>
#include <ze/common/types.hpp>

namespace ze {

// Oldest entry: buffer.begin(), newest entry: buffer.rbegin()
template <typename Scalar, int Dim> class Buffer {
public:
  using Vector = Eigen::Matrix<Scalar, Dim, 1>;
  using VectorBuffer = std::map<int64_t, Vector, std::less<int64_t>,
                                Eigen::aligned_allocator<Vector>>;

  static constexpr int kDim = Dim;

  Buffer() = default;
  Buffer(real_t buffer_size_seconds)
      : buffer_size_nanosec_(secToNanosec(buffer_size_seconds)) {}

  Buffer(const Buffer &from) {
    std::lock_guard<std::mutex>(from.mutex_);
    if (from.buffer_.size() != 0) {
      throw std::runtime_error("Non-empty buffers are not copyable.");
    }
    buffer_size_nanosec_ = from.buffer_size_nanosec_;
  }

  inline void insert(int64_t stamp, const Vector &data) {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_[stamp] = data;
    if (buffer_size_nanosec_ > 0) {
      removeDataBeforeTimestamp_impl(buffer_.rbegin()->first -
                                     buffer_size_nanosec_);
    }
  }

  //! Get value with timestamp closest to stamp. Boolean in returns if
  //! successful.
  std::tuple<int64_t, Vector, bool> getNearestValue(int64_t stamp);

  //! Get oldest value in buffer.
  std::pair<Vector, bool> getOldestValue() const;

  //! Get newest value in buffer.
  std::pair<Vector, bool> getNewestValue() const;

  //! Get timestamps of newest and oldest entry.
  std::tuple<int64_t, int64_t, bool> getOldestAndNewestStamp() const;

  /*! @brief Get Values between timestamps.
   *
   * If timestamps are not matched, the values
   * are interpolated. Returns a vector of timestamps and a block matrix with
   * values as columns. Returns empty matrices if not successful.
   */
  std::pair<Eigen::Matrix<int64_t, Eigen::Dynamic, 1>,
            Eigen::Matrix<Scalar, Dim, Eigen::Dynamic>>
  getBetweenValuesInterpolated(int64_t stamp_from, int64_t stamp_to);

  inline void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
  }

  inline size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.size();
  }

  inline bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.empty();
  }

  inline void removeDataBeforeTimestamp(int64_t stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    removeDataBeforeTimestamp_impl(stamp);
  }

  inline void removeDataOlderThan(double seconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.empty())
      return;

    removeDataBeforeTimestamp_impl(buffer_.rbegin()->first -
                                   secToNanosec(seconds));
  }

  inline void lock() const { mutex_.lock(); }

  inline void unlock() const { mutex_.unlock(); }

  const VectorBuffer &data() const {
    CHECK(!mutex_.try_lock()) << "Call lock() before accessing data.";
    return buffer_;
  }

  typename VectorBuffer::iterator iterator_equal_or_before(int64_t stamp);

  typename VectorBuffer::iterator iterator_equal_or_after(int64_t stamp);

protected:
  mutable std::mutex mutex_;
  VectorBuffer buffer_;
  int64_t buffer_size_nanosec_ = -1; // Negative means, no fixed size.

  inline void removeDataBeforeTimestamp_impl(int64_t stamp) {
    auto it = buffer_.lower_bound(stamp);
    buffer_.erase(buffer_.begin(), it);
  }
};

// -----------------------------------------------------------------------------
template <typename BuffScalar, int BuffDim>
bool findNearestTimeStamp(Buffer<BuffScalar, BuffDim> &in_buff,
                          const int64_t &in_ts, int64_t &out_ts,
                          Eigen::Matrix<BuffScalar, BuffDim, 1> &out_data,
                          real_t max_diff_secs = 0.02,
                          real_t offset_secs = 0.0) {
  bool success;
  int64_t offset_nsecs = secToNanosec(offset_secs);
  std::tie(out_ts, out_data, success) =
      in_buff.getNearestValue(in_ts + offset_nsecs);
  if (!success) {
    return false;
  }
  int64_t max_diff_nsecs = secToNanosec(max_diff_secs);
  if (std::abs(out_ts - in_ts) > max_diff_nsecs) {
    return false;
  } else
    return true;
}

} // namespace ze

#include <ze/common/buffer-inl.hpp>
