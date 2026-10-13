// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "allocation_failure.hpp"

#include <cstdlib>
#include <new>

namespace volumetric_kit::calib::test {
thread_local AllocationFailure allocation_failure;
}  // namespace volumetric_kit::calib::test

void* operator new(std::size_t bytes) {
  auto& failure = volumetric_kit::calib::test::allocation_failure;
  if (failure.armed) {
    if (failure.remaining == 0) {
      failure.armed = false;  // The OutOfMemory message can allocate.
      failure.injected = true;
      throw std::bad_alloc();
    }
    --failure.remaining;
  }
  if (void* p = std::malloc(bytes == 0 ? 1 : bytes)) return p;
  throw std::bad_alloc();
}

void operator delete(void* p) noexcept { std::free(p); }
void* operator new[](std::size_t bytes) { return ::operator new(bytes); }
void operator delete[](void* p) noexcept { ::operator delete(p); }
// Linked libraries may use sized delete even when this compiler does not.
void operator delete(void* p, std::size_t) noexcept { ::operator delete(p); }
void operator delete[](void* p, std::size_t) noexcept {
  ::operator delete[](p);
}

// Keep nothrow allocation paired with the same deallocator under sanitizers.
void* operator new(std::size_t bytes, const std::nothrow_t&) noexcept {
  try {
    return ::operator new(bytes);
  } catch (const std::bad_alloc&) {
    return nullptr;
  }
}
void* operator new[](std::size_t bytes, const std::nothrow_t&) noexcept {
  return ::operator new(bytes, std::nothrow);
}
void operator delete(void* p, const std::nothrow_t&) noexcept {
  ::operator delete(p);
}
void operator delete[](void* p, const std::nothrow_t&) noexcept {
  ::operator delete[](p);
}
