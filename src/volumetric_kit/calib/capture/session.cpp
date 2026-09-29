// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/capture/session.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "png_io.hpp"

// Everything here is exception-free, since the library is also built with
// -fno-exceptions: the JSON is parsed with exceptions off and every value is
// type-checked before it is read, and std::filesystem is called through its
// std::error_code overloads.

namespace volumetric_kit::calib::capture {
namespace {

namespace fs = std::filesystem;
using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;

constexpr const char* kFormat = "volumetric_kit_calib.session";
constexpr int kVersion = 1;
constexpr const char* kSessionFile = "session.json";

std::string session_file(const std::string& dir) {
  return (fs::path(dir) / kSessionFile).string();
}

// --- Writing -------------------------------------------------------------

ordered_json lens_json(const PinholeBrownConrady& lens) {
  const PinholeBrownConrady::Intrinsics& k = lens.intrinsics();
  const PinholeBrownConrady::Distortion& d = lens.distortion();
  ordered_json j;
  j["width"] = lens.image_size().width;
  j["height"] = lens.image_size().height;
  j["fx"] = k.fx;
  j["fy"] = k.fy;
  j["cx"] = k.cx;
  j["cy"] = k.cy;
  j["k1"] = d.k1;
  j["k2"] = d.k2;
  j["p1"] = d.p1;
  j["p2"] = d.p2;
  j["k3"] = d.k3;
  return j;
}

std::string session_text(const Session& session) {
  ordered_json doc;
  doc["format"] = kFormat;
  doc["version"] = kVersion;
  ordered_json cameras = ordered_json::array();
  for (const CameraInfo& camera : session.cameras) {
    ordered_json c;
    c["serial"] = camera.serial;
    c["role"] = to_string(camera.role);
    c["lens"] = lens_json(*camera.lens);  // every session camera has one
    cameras.push_back(std::move(c));
  }
  doc["cameras"] = std::move(cameras);
  ordered_json captures = ordered_json::array();
  for (const Capture& capture : session.captures) {
    ordered_json c;
    c["index"] = static_cast<std::uint64_t>(capture.index);
    c["trigger_ns"] = capture.trigger_ns;
    c["frames_averaged"] = capture.frames_averaged;
    captures.push_back(std::move(c));
  }
  doc["captures"] = std::move(captures);
  // Serials are plain ASCII (is_session_serial), so `replace` never fires; it
  // is the dump that cannot throw.
  return doc.dump(2, ' ', false, ordered_json::error_handler_t::replace) + "\n";
}

// Write `session.json` through a temporary renamed into place, so a reader
// never sees half a file.
Status write_session_file(const std::string& dir, const Session& session) {
  const std::string path = session_file(dir);
  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out) return Status::io_error(tmp + ": cannot be opened for writing");
    out << session_text(session);
    out.flush();
    if (!out) return Status::io_error(tmp + ": write failed");
  }
  std::error_code ec;
  fs::rename(tmp, path, ec);
  if (ec) {
    fs::remove(tmp, ec);
    return Status::io_error(path + ": cannot be replaced");
  }
  return {};
}

// --- Reading -------------------------------------------------------------

// The value at `key` of object `obj`, or null when absent.
const json* member(const json& obj, const char* key) {
  const auto it = obj.find(key);
  return it == obj.end() ? nullptr : &*it;
}

Status bad(const std::string& where, const std::string& what) {
  return Status::invalid_argument(where + ": " + what);
}

Result<double> number_at(const json& obj, const char* key,
                         const std::string& where) {
  const json* v = member(obj, key);
  if (v == nullptr || !v->is_number()) {
    return bad(where, std::string(key) + " is missing or not a number");
  }
  return v->get<double>();
}

Result<std::uint64_t> unsigned_at(const json& obj, const char* key,
                                  const std::string& where) {
  const json* v = member(obj, key);
  if (v == nullptr || !v->is_number_unsigned()) {
    return bad(where,
               std::string(key) + " is missing or not a non-negative integer");
  }
  return v->get<std::uint64_t>();
}

Result<std::string> string_at(const json& obj, const char* key,
                              const std::string& where) {
  const json* v = member(obj, key);
  if (v == nullptr || !v->is_string()) {
    return bad(where, std::string(key) + " is missing or not a string");
  }
  return v->get<std::string>();
}

Result<int> size_at(const json& obj, const char* key,
                    const std::string& where) {
  VC_ASSIGN(const std::uint64_t n, unsigned_at(obj, key, where));
  if (n == 0 ||
      n > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
    return bad(where, std::string(key) + " is out of range");
  }
  return static_cast<int>(n);
}

Result<SyncRole> parse_role(const std::string& text, const std::string& where) {
  for (const SyncRole role :
       {SyncRole::Standalone, SyncRole::Primary, SyncRole::Secondary}) {
    if (text == to_string(role)) return role;
  }
  return bad(where, "role \"" + text +
                        "\" is not one of standalone, "
                        "primary, secondary");
}

Result<PinholeBrownConrady> parse_lens(const json& j,
                                       const std::string& where) {
  if (!j.is_object()) return bad(where, "is not an object");
  VC_ASSIGN(const int width, size_at(j, "width", where));
  VC_ASSIGN(const int height, size_at(j, "height", where));
  PinholeBrownConrady::Intrinsics k;
  VC_ASSIGN(k.fx, number_at(j, "fx", where));
  VC_ASSIGN(k.fy, number_at(j, "fy", where));
  VC_ASSIGN(k.cx, number_at(j, "cx", where));
  VC_ASSIGN(k.cy, number_at(j, "cy", where));
  PinholeBrownConrady::Distortion d;
  VC_ASSIGN(d.k1, number_at(j, "k1", where));
  VC_ASSIGN(d.k2, number_at(j, "k2", where));
  VC_ASSIGN(d.p1, number_at(j, "p1", where));
  VC_ASSIGN(d.p2, number_at(j, "p2", where));
  VC_ASSIGN(d.k3, number_at(j, "k3", where));
  Result<PinholeBrownConrady> lens =
      PinholeBrownConrady::create({width, height}, k, d);
  if (!lens) return bad(where, lens.status().message());
  return lens;
}

Result<Session> parse_session(const std::string& text,
                              const std::string& file) {
  const json doc = json::parse(text, nullptr, /*allow_exceptions=*/false);
  if (doc.is_discarded()) return bad(file, "is not JSON");
  if (!doc.is_object()) return bad(file, "is not a JSON object");
  {
    const json* format = member(doc, "format");
    if (format == nullptr || !format->is_string() ||
        format->get<std::string>() != kFormat) {
      return bad(file, std::string("format is not \"") + kFormat + "\"");
    }
    const json* version = member(doc, "version");
    if (version == nullptr || !version->is_number_integer() ||
        version->get<std::int64_t>() != kVersion) {
      return bad(file, "version is not " + std::to_string(kVersion));
    }
  }

  Session session;
  const json* cameras = member(doc, "cameras");
  if (cameras == nullptr || !cameras->is_array() || cameras->empty()) {
    return bad(file, "cameras is missing, not an array, or empty");
  }
  for (std::size_t i = 0; i < cameras->size(); ++i) {
    const json& c = (*cameras)[i];
    const std::string where = file + ": camera " + std::to_string(i);
    if (!c.is_object()) return bad(where, "is not an object");
    CameraInfo camera;
    VC_ASSIGN(camera.serial, string_at(c, "serial", where));
    if (!is_session_serial(camera.serial)) {
      return bad(where,
                 "serial \"" + camera.serial + "\" is not a plain file name");
    }
    for (const CameraInfo& earlier : session.cameras) {
      if (earlier.serial == camera.serial) {
        return bad(where, "serial " + camera.serial + " is repeated");
      }
    }
    VC_ASSIGN(const std::string role, string_at(c, "role", where));
    VC_ASSIGN(camera.role, parse_role(role, where));
    const json* lens = member(c, "lens");
    if (lens == nullptr) return bad(where, "has no lens");
    VC_ASSIGN(PinholeBrownConrady model, parse_lens(*lens, where + ": lens"));
    camera.lens = std::move(model);
    session.cameras.push_back(std::move(camera));
  }

  const json* captures = member(doc, "captures");
  if (captures == nullptr || !captures->is_array()) {
    return bad(file, "captures is missing or not an array");
  }
  for (std::size_t i = 0; i < captures->size(); ++i) {
    const json& c = (*captures)[i];
    const std::string where = file + ": capture " + std::to_string(i);
    if (!c.is_object()) return bad(where, "is not an object");
    Capture capture;
    VC_ASSIGN(const std::uint64_t index, unsigned_at(c, "index", where));
    if (index > std::numeric_limits<std::size_t>::max()) {
      return bad(where, "index is out of range");
    }
    capture.index = static_cast<std::size_t>(index);
    if (!session.captures.empty() &&
        capture.index <= session.captures.back().index) {
      return bad(where, "index " + std::to_string(capture.index) +
                            " is not above the capture before it");
    }
    VC_ASSIGN(capture.trigger_ns, unsigned_at(c, "trigger_ns", where));
    VC_ASSIGN(const std::uint64_t averaged,
              unsigned_at(c, "frames_averaged", where));
    if (averaged == 0 || averaged > std::numeric_limits<std::uint32_t>::max()) {
      return bad(where, "frames_averaged is out of range");
    }
    capture.frames_averaged = static_cast<std::uint32_t>(averaged);
    session.captures.push_back(capture);
  }
  return session;
}

}  // namespace

bool is_session_serial(const std::string& serial) noexcept {
  if (serial.empty() || serial == "." || serial == "..") return false;
  for (const char ch : serial) {
    const bool plain = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                       (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' ||
                       ch == '.';
    if (!plain) return false;
  }
  return true;
}

std::string session_image_path(const std::string& dir,
                               const std::string& serial, std::size_t index) {
  return (fs::path(dir) / serial /
          ("calib_img_" + serial + "_" + std::to_string(index) + ".png"))
      .string();
}

Result<Session> read_session(const std::string& dir) {
  const std::string path = session_file(dir);
  std::error_code ec;
  if (!fs::exists(path, ec)) return Status::not_found(path + ": not found");
  std::ifstream in(path, std::ios::binary);
  if (!in) return Status::io_error(path + ": cannot be opened");
  std::ostringstream text;
  text << in.rdbuf();
  if (in.bad()) return Status::io_error(path + ": read failed");
  return parse_session(text.str(), path);
}

Result<LumaImage> read_session_image(const std::string& dir,
                                     const std::string& serial,
                                     std::size_t index) {
  const std::string path = session_image_path(dir, serial, index);
  std::error_code ec;
  if (!fs::exists(path, ec)) return Status::not_found(path + ": not found");
  return detail::read_png(path);
}

Result<SessionWriter> SessionWriter::create(std::string dir,
                                            std::vector<CameraInfo> cameras) {
  if (cameras.empty()) return Status::invalid_argument("no cameras");
  for (std::size_t i = 0; i < cameras.size(); ++i) {
    const CameraInfo& camera = cameras[i];
    const std::string where = "camera " + std::to_string(i);
    if (!is_session_serial(camera.serial)) {
      return bad(where,
                 "serial \"" + camera.serial + "\" is not a plain file name");
    }
    if (!camera.lens) return bad(where, camera.serial + " has no lens");
    for (std::size_t j = 0; j < i; ++j) {
      if (cameras[j].serial == camera.serial) {
        return bad(where, "serial " + camera.serial + " is repeated");
      }
    }
  }
  std::error_code ec;
  if (fs::exists(session_file(dir), ec)) {
    return Status::invalid_argument(dir + ": already holds a session");
  }
  for (const CameraInfo& camera : cameras) {
    const fs::path camera_dir = fs::path(dir) / camera.serial;
    fs::create_directories(camera_dir, ec);
    if (ec) {
      return Status::io_error(camera_dir.string() + ": cannot be created (" +
                              ec.message() + ")");
    }
  }
  Session session;
  session.cameras = std::move(cameras);
  VC_TRY(write_session_file(dir, session));
  return SessionWriter{std::move(dir), std::move(session)};
}

Result<std::size_t> SessionWriter::add(std::uint64_t trigger_ns,
                                       std::uint32_t frames_averaged,
                                       const std::vector<LumaImage>& images) {
  const std::vector<CameraInfo>& cameras = session_.cameras;
  if (frames_averaged == 0) {
    return Status::invalid_argument("a capture averages at least one frame");
  }
  if (images.size() != cameras.size()) {
    return Status::invalid_argument(
        std::to_string(images.size()) + " images for " +
        std::to_string(cameras.size()) + " cameras");
  }
  for (std::size_t i = 0; i < cameras.size(); ++i) {
    const ImageSize& want = cameras[i].lens->image_size();
    const LumaImage& image = images[i];
    const std::size_t pixels = static_cast<std::size_t>(want.width) *
                               static_cast<std::size_t>(want.height);
    if (image.size.width != want.width || image.size.height != want.height ||
        image.pixels.size() != pixels) {
      return Status::invalid_argument(
          cameras[i].serial + ": image is " + std::to_string(image.size.width) +
          "x" + std::to_string(image.size.height) + " with " +
          std::to_string(image.pixels.size()) + " pixels, its lens " +
          std::to_string(want.width) + "x" + std::to_string(want.height));
    }
  }

  const std::size_t index =
      session_.captures.empty() ? 0 : session_.captures.back().index + 1;
  std::vector<std::string> written;
  const auto unwind = [&written]() {
    std::error_code ec;
    for (const std::string& path : written) fs::remove(path, ec);
  };
  for (std::size_t i = 0; i < cameras.size(); ++i) {
    const std::string path = session_image_path(dir_, cameras[i].serial, index);
    const Status status = detail::write_png(path, images[i]);
    if (!status) {
      unwind();
      return status;
    }
    written.push_back(path);
  }

  session_.captures.push_back({index, trigger_ns, frames_averaged});
  const Status status = write_session_file(dir_, session_);
  if (!status) {
    session_.captures.pop_back();
    unwind();
    return status;
  }
  return index;
}

Status SessionWriter::remove_last() {
  if (session_.captures.empty()) {
    return Status::not_found(dir_ + ": no capture to remove");
  }
  // The listing first, so the session never names a capture it lacks images
  // for; an image left behind is overwritten by the next add, which reuses
  // the index.
  const Capture last = session_.captures.back();
  session_.captures.pop_back();
  const Status status = write_session_file(dir_, session_);
  if (!status) {
    session_.captures.push_back(last);
    return status;
  }
  std::string left;
  for (const CameraInfo& camera : session_.cameras) {
    const std::string path =
        session_image_path(dir_, camera.serial, last.index);
    std::error_code ec;
    fs::remove(path, ec);
    if (ec) left += (left.empty() ? "" : ", ") + path;
  }
  if (!left.empty()) {
    return Status::io_error("capture removed, but not its images: " + left);
  }
  return {};
}

}  // namespace volumetric_kit::calib::capture
