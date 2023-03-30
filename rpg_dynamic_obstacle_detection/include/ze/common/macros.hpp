// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <memory>

//! @file macros.hpp
//! Common macros such as ZE_POINTER_TYPEDEFS, ZE_DELETE_COPY_ASSIGN, UNLIKELY.

#define ZE_POINTER_TYPEDEFS(TypeName)                                          \
  typedef std::unique_ptr<TypeName> UniquePtr;                                 \
  typedef std::shared_ptr<TypeName> Ptr;                                       \
  typedef std::shared_ptr<const TypeName> ConstPtr

#define ZE_DELETE_COPY_ASSIGN(TypeName)                                        \
  TypeName(const TypeName &) = delete;                                         \
  void operator=(const TypeName &) = delete

// Give the compiler a hint that an if statement is likely true or false.
// You should use it only in cases when the likeliest branch is very very very
// likely, or when the unlikeliest branch is very very very unlikely.
#define UNLIKELY(x) __builtin_expect((bool)(x), 0)
#define LIKELY(x) __builtin_expect((bool)(x), 1)
