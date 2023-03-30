// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/types.hpp>

namespace ze {

//! A template for a data handle that is composed of the slot index (in an
//! array) plus a version number. The indexed array stores the version number of
//! each object and can therefore tell if the object at the slot is at the same
//! version we request.
//! http://seanmiddleditch.com/data-structures-for-game-developers-the-slot-map/
//! http://blog.molecular-matters.com/2013/05/17/adventures-in-data-oriented-design-part-3b-internal-references/
template <typename T, int NumSlotBits, int NumVersionBits>
union VersionedSlotHandle {
  using value_t = T;

  struct {
    // The first NumSlotBits in this bit-field store the slot, the next
    // NumVersionBits store the version number of the handle.
    T slot : NumSlotBits;
    T version : NumVersionBits;
  };
  T handle;

  // Always initialize handle first to zero to make sure that all bits are
  // really 0.
  VersionedSlotHandle() : handle(0) {}
  VersionedSlotHandle(T handle) : handle(handle) {}
  VersionedSlotHandle(T _slot, T _version) : handle(0) {
    slot = _slot;
    version = _version;
  }

  static constexpr T maxSlot() { return (1 << NumSlotBits) - 1; }
  static constexpr T maxVersion() { return (1 << NumVersionBits) - 1; }

  void reset() { handle = T{0}; }
};

template <typename T, int NumSlotBits, int NumVersionBits>
inline bool
operator==(const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> lhs,
           const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> rhs) {
  return lhs.handle == rhs.handle;
}

template <typename T, int NumSlotBits, int NumVersionBits>
inline bool
operator!=(const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> lhs,
           const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> rhs) {
  return !(lhs.handle == rhs.handle);
}

template <typename T, int NumSlotBits, int NumVersionBits>
inline std::ostream &
operator<<(std::ostream &out,
           const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> handle) {
  out << handle.handle << " (slot: " << handle.slot
      << ", version: " << handle.version << ")";
  return out;
}

//! Compares the handle as an integer. I.e. Compare slot number first, then
//! version.
template <typename T, int NumSlotBits, int NumVersionBits>
inline bool
operator<(const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> lhs,
          const VersionedSlotHandle<T, NumSlotBits, NumVersionBits> rhs) {
  return lhs.handle < rhs.handle;
}

} // namespace ze
