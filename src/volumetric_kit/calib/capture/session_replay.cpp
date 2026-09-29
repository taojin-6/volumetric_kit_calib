// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/capture/session_replay.hpp"

#include <string>
#include <utility>
#include <vector>

namespace volumetric_kit::calib::capture {

Result<SessionReplay> SessionReplay::open(std::string dir) {
  VC_ASSIGN(Session session, read_session(dir));
  return SessionReplay{std::move(dir), std::move(session)};
}

Status SessionReplay::start() {
  started_ = true;
  next_ = 0;
  current_.reset();
  return {};
}

void SessionReplay::stop() noexcept {
  started_ = false;
  current_.reset();
}

Result<std::optional<SetInfo>> SessionReplay::poll_set() {
  if (!started_ || next_ >= session_.captures.size()) return no_set();
  SetInfo set;
  set.trigger_ns = session_.captures[next_].trigger_ns;
  set.present.assign(session_.cameras.size(), true);
  current_ = next_++;
  return some_set(std::move(set));
}

Result<LumaImage> SessionReplay::read_luma(std::size_t camera) {
  if (!started_ || !current_) {
    return Status::invalid_argument(
        "read_luma: no set has been polled since start");
  }
  if (camera >= session_.cameras.size()) {
    return Status::invalid_argument("read_luma: camera " +
                                    std::to_string(camera) + " of " +
                                    std::to_string(session_.cameras.size()));
  }
  const CameraInfo& info = session_.cameras[camera];
  VC_ASSIGN(LumaImage image,
            read_session_image(dir_, info.serial,
                               session_.captures[*current_].index));
  const ImageSize& want = info.lens->image_size();
  if (image.size.width != want.width || image.size.height != want.height) {
    return Status::invalid_argument(
        info.serial + ": image is " + std::to_string(image.size.width) + "x" +
        std::to_string(image.size.height) + ", its lens " +
        std::to_string(want.width) + "x" + std::to_string(want.height));
  }
  return image;
}

bool SessionReplay::exhausted() const noexcept {
  return started_ && next_ >= session_.captures.size();
}

}  // namespace volumetric_kit::calib::capture
