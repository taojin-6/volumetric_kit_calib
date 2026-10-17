// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "allocation_failure.hpp"

#include <dlfcn.h>
#include <execinfo.h>

#include <cstdlib>
#include <cstring>
#include <new>

namespace volumetric_kit::calib::test {
thread_local AllocationFailure allocation_failure;
}  // namespace volumetric_kit::calib::test

namespace {

// Whether a frame of the calling thread's stack is in the Vulkan loader
// (libvulkan*), a layer (*VkLayer*) or MoltenVK. Neither call reaches
// operator new.
bool under_vulkan() noexcept {
  void* frames[128];
  const int depth = backtrace(frames, 128);
  for (int i = 0; i < depth; ++i) {
    Dl_info info;
    if (dladdr(frames[i], &info) == 0 || info.dli_fname == nullptr) continue;
    const char* slash = std::strrchr(info.dli_fname, '/');
    const char* name = slash == nullptr ? info.dli_fname : slash + 1;
    if (std::strncmp(name, "libvulkan", 9) == 0 ||
        std::strstr(name, "VkLayer") != nullptr ||
        std::strstr(name, "MoltenVK") != nullptr) {
      return true;
    }
  }
  return false;
}

}  // namespace

void* operator new(std::size_t bytes) {
  auto& failure = volumetric_kit::calib::test::allocation_failure;
  if (failure.armed && !(failure.spare_vulkan && under_vulkan())) {
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

// A nothrow allocation is neither counted nor failed: its caller copes with a
// null pointer itself -- libstdc++'s stable_sort, refused its buffer, sorts in
// place -- so failing one would test the standard library, not calib. Paired
// with the same deallocator under sanitizers.
void* operator new(std::size_t bytes, const std::nothrow_t&) noexcept {
  return std::malloc(bytes == 0 ? 1 : bytes);
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
