// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// calib's JSON text, written in order, by hand: private and header-only. A
// tier that writes a JSON file includes it with nlohmann/json on its private
// include path. nlohmann makes each number and string, and none of its
// containers, whose destructors allocate and so, out of memory, terminate
// (DECISIONS.md, 2026-10-16).

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace volumetric_kit::calib::detail {

// A double in the shortest form that reads back to it, as nlohmann/json
// writes one; `null` for one not finite.
inline std::string json_number(double v) { return nlohmann::json(v).dump(); }

// A string quoted and escaped. An id from a driver may not be UTF-8: its bad
// bytes become U+FFFD rather than a throw.
inline std::string json_quoted(std::string_view s) {
  return nlohmann::json(std::string(s))
      .dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
}

// A JSON document, written in order. Objects and arrays open on the line they
// are named on, each member on a line of its own, indented by 2; a flat array
// keeps its elements on one line.
//
//   JsonWriter w;
//   w.open_object();
//   w.key("format").value("volumetric_kit_calib.report");
//   w.key("counts").open_array(true);
//   w.value(std::size_t{3});
//   w.close_array();
//   w.close_object();
//   const std::string text = std::move(w).text();
class JsonWriter {
 public:
  void open_object() { open('{', false); }
  void open_array(bool flat = false) { open('[', flat); }
  void close_object() { close('}'); }
  void close_array() { close(']'); }

  JsonWriter& key(std::string_view name) {
    before_value();
    text_ += json_quoted(name);
    text_ += ": ";
    after_key_ = true;
    return *this;
  }

  void value(double v) {
    before_value();
    text_ += json_number(v);
  }
  void value(std::int64_t v) {
    before_value();
    text_ += std::to_string(v);
  }
  void value(int v) { value(static_cast<std::int64_t>(v)); }
  void value(std::size_t v) {
    before_value();
    text_ += std::to_string(v);
  }
  void value(bool v) {
    before_value();
    text_ += v ? "true" : "false";
  }
  void value(std::string_view v) {
    before_value();
    text_ += json_quoted(v);
  }
  void value(const char* v) { value(std::string_view(v)); }
  void null() {
    before_value();
    text_ += "null";
  }

  // The finished document, with a final line break.
  std::string text() && { return std::move(text_) + "\n"; }

 private:
  struct Scope {
    bool flat = false;
    bool empty = true;
  };

  void newline(std::size_t depth) {
    text_ += '\n';
    text_.append(2 * depth, ' ');
  }

  void before_value() {
    if (after_key_) {
      after_key_ = false;
      return;
    }
    if (scopes_.empty()) return;
    Scope& scope = scopes_.back();
    if (!scope.empty) text_ += scope.flat ? ", " : ",";
    if (!scope.flat) newline(scopes_.size());
    scope.empty = false;
  }

  void open(char bracket, bool flat) {
    before_value();
    text_ += bracket;
    scopes_.push_back({flat || (!scopes_.empty() && scopes_.back().flat)});
  }

  void close(char bracket) {
    const Scope scope = scopes_.back();
    scopes_.pop_back();
    if (!scope.empty && !scope.flat) newline(scopes_.size());
    text_ += bracket;
  }

  std::string text_;
  std::vector<Scope> scopes_;
  bool after_key_ = false;
};

}  // namespace volumetric_kit::calib::detail
