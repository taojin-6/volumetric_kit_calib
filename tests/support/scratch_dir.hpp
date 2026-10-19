// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file scratch_dir.hpp
/// @brief A test's own directory on disk, for the tests that write files,
///        and what a file there holds.

#include <unistd.h>

#include <atomic>
#include <filesystem>
#include <string>
#include <system_error>
#include <utility>

#include <gtest/gtest.h>

#include "volumetric_kit/core/base/file_io.hpp"

namespace volumetric_kit::calib::test {

/// @brief What a file holds, failing the test when it cannot be read.
/// @param path  The file.
/// @return Its bytes; empty when it cannot be read.
inline std::string read(const std::filesystem::path& path) {
  core::Result<std::string> bytes = core::read_file(path.string());
  EXPECT_TRUE(bytes.ok()) << bytes.status().message();
  return bytes.ok() ? std::move(bytes).value() : std::string();
}

/// @brief A fresh directory under the system's temporary one, named for the
///        running test, removed with what it holds when it goes.
///
/// @code
/// const ScratchDir dir;
/// write_session(dir / "session");  // dir.path() / "session"
/// @endcode
class ScratchDir {
 public:
  /// @brief Make the directory, empty.
  ScratchDir() {
    static std::atomic<int> counter{0};
    const auto* test = ::testing::UnitTest::GetInstance()->current_test_info();
    std::error_code ec;
    path_ =
        std::filesystem::temp_directory_path(ec) /
        ("vc_" + std::to_string(::getpid()) + "_" + test->test_suite_name() +
         "_" + test->name() + "_" + std::to_string(counter++));
    std::filesystem::remove_all(path_, ec);
    std::filesystem::create_directories(path_, ec);
  }
  /// @brief Remove it and what it holds, a directory the test left
  ///        read-only included.
  ~ScratchDir() {
    namespace fs = std::filesystem;
    std::error_code ec;
    for (auto it = fs::recursive_directory_iterator(path_, ec);
         it != fs::recursive_directory_iterator(); it.increment(ec)) {
      if (it->is_directory(ec)) {
        fs::permissions(it->path(), fs::perms::owner_all, fs::perm_options::add,
                        ec);
      }
    }
    fs::remove_all(path_, ec);
  }
  ScratchDir(const ScratchDir&) = delete;
  ScratchDir& operator=(const ScratchDir&) = delete;

  /// @return The directory.
  const std::filesystem::path& path() const { return path_; }
  /// @param name  An entry's name.
  /// @return Its path in the directory.
  std::string operator/(const std::string& name) const {
    return (path_ / name).string();
  }

 private:
  std::filesystem::path path_;
};

}  // namespace volumetric_kit::calib::test
