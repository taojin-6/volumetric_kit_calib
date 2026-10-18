// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "allocation_failure.hpp"

#include <dlfcn.h>
#include <execinfo.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <numeric>
#include <unordered_set>
#include <vector>

namespace volumetric_kit::calib::test {
thread_local AllocationFailure allocation_failure;
}  // namespace volumetric_kit::calib::test

namespace {

namespace test = volumetric_kit::calib::test;

// Storage from malloc, so recording an allocation makes none.
template <class T>
struct MallocAllocator {
  using value_type = T;
  MallocAllocator() = default;
  template <class U>
  MallocAllocator(const MallocAllocator<U>&) noexcept {}
  T* allocate(std::size_t n) {
    void* p = std::malloc(n * sizeof(T));
    if (p == nullptr) std::abort();
    return static_cast<T*>(p);
  }
  void deallocate(T* p, std::size_t) noexcept { std::free(p); }
  friend bool operator==(MallocAllocator, MallocAllocator) noexcept {
    return true;
  }
  friend bool operator!=(MallocAllocator, MallocAllocator) noexcept {
    return false;
  }
};

// The census's call stacks, one for each allocation in order.
thread_local std::vector<std::uint64_t, MallocAllocator<std::uint64_t>>
    census_stacks;

constexpr int kMaxFrames = 128;

// The calling thread's return addresses, innermost first. Neither backtrace
// nor dladdr reaches operator new.
struct Stack {
  Stack() noexcept : depth(backtrace(frames, kMaxFrames)) {}

  void* frames[kMaxFrames];
  int depth;

  // Whether a frame is in the Vulkan loader (libvulkan*), a layer
  // (*VkLayer*) or MoltenVK.
  bool under_vulkan() const noexcept {
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

  // The return addresses, hashed (splitmix64's finaliser over each).
  std::uint64_t id() const noexcept {
    std::uint64_t h = 0x9e3779b97f4a7c15u;
    for (int i = 0; i < depth; ++i) {
      std::uint64_t x = h + reinterpret_cast<std::uintptr_t>(frames[i]);
      x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9u;
      x = (x ^ (x >> 27)) * 0x94d049bb133111ebu;
      h = x ^ (x >> 31);
    }
    return h;
  }
};

bool exhaustive() {
  const char* every = std::getenv("VC_TEST_OOM_EXHAUSTIVE");
  return every != nullptr && std::strcmp(every, "1") == 0;
}

}  // namespace

namespace volumetric_kit::calib::test::detail {

void begin_census() noexcept { census_stacks.clear(); }

std::vector<std::size_t> end_census(std::size_t& count) {
  count = census_stacks.size();
  std::vector<std::size_t> plan;
  if (exhaustive()) {
    plan.resize(count);
    std::iota(plan.begin(), plan.end(), std::size_t{0});
    return plan;
  }
  std::unordered_set<std::uint64_t> seen;
  for (std::size_t i = 0; i < count; ++i) {
    if (seen.insert(census_stacks[i]).second) plan.push_back(i);
  }
  return plan;
}

std::uint64_t census_stack(std::size_t index) noexcept {
  return index < census_stacks.size() ? census_stacks[index] : 0;
}

}  // namespace volumetric_kit::calib::test::detail

// Armed, the stack is walked only where needed: for each allocation of a
// census or while the Vulkan driver is spared, and for the one failed.
void* operator new(std::size_t bytes) {
  auto& failure = test::allocation_failure;
  if (failure.armed) {
    if (!failure.census && !failure.spare_vulkan && failure.remaining > 0) {
      --failure.remaining;
    } else {
      const Stack stack;
      if (!(failure.spare_vulkan && stack.under_vulkan())) {
        if (failure.census) {
          census_stacks.push_back(stack.id());
        } else if (failure.remaining > 0) {
          --failure.remaining;
        } else {
          failure.armed = false;  // The OutOfMemory message can allocate.
          failure.injected = true;
          failure.failed_stack = stack.id();
          throw std::bad_alloc();
        }
      }
    }
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
