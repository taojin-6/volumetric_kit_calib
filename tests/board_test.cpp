// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The ChArUco board: OpenCV's board for a Target, its frame and corners, each
// dictionary's codes, and the targets it refuses.

#include "volumetric_kit/calib/compute/board.hpp"

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/objdetect/aruco_dictionary.hpp>

namespace volumetric_kit::calib::compute {
namespace {

// The status is `code`, with a message naming `target.<field>`.
::testing::AssertionResult RefusedNaming(const core::Status& status,
                                         core::Status::Code code,
                                         const std::string& field) {
  if (status.ok()) return ::testing::AssertionFailure() << "accepted";
  if (status.domain() != code) {
    return ::testing::AssertionFailure()
           << "refused as " << core::to_string(status.domain()) << ": "
           << status.message();
  }
  if (status.message().rfind("target." + field + " ", 0) != 0) {
    return ::testing::AssertionFailure()
           << "message is not about " << field << ": " << status.message();
  }
  return ::testing::AssertionSuccess();
}

core::Status create_status(const Target& target) {
  core::Result<Board> board = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(board = Board::create(target));
  return board.ok() ? core::Status{} : board.status();
}

TEST(Board, IsTheRigsBoard) {
  const core::Result<Board> board = Board::create(legacy_femto_board());
  ASSERT_TRUE(board.ok()) << board.status().message();
  const cv::aruco::CharucoBoard& charuco = board->charuco();
  EXPECT_EQ(charuco.getChessboardSize(), cv::Size(5, 3));
  EXPECT_EQ(charuco.getSquareLength(), 0.15f);
  EXPECT_EQ(charuco.getMarkerLength(), 0.112f);
  EXPECT_TRUE(charuco.getLegacyPattern());
  EXPECT_EQ(charuco.getIds(), (std::vector<int>{0, 1, 2, 3, 4, 5, 6}));
  EXPECT_EQ(charuco.getDictionary().markerSize, 4);
  EXPECT_EQ(charuco.getDictionary().bytesList.rows, 100);
  EXPECT_EQ(board->target().rows, 2);
}

// The board frame: the origin at the top-left outer corner, x right, y down,
// the corners numbered row by row.
TEST(Board, CornersSitOnTheGridFromTheTopLeft) {
  const core::Result<Board> board = Board::create(legacy_femto_board());
  ASSERT_TRUE(board.ok()) << board.status().message();
  const std::vector<cv::Point3d>& corners = board->object_points();
  ASSERT_EQ(corners.size(), 8u);
  const double s = 0.15;
  for (int i = 0; i < 8; ++i) {
    // OpenCV keeps the board in float.
    EXPECT_NEAR(corners[i].x, ((i % 4) + 1) * s, 1e-7) << i;
    EXPECT_NEAR(corners[i].y, ((i / 4) + 1) * s, 1e-7) << i;
    EXPECT_EQ(corners[i].z, 0.0) << i;
  }
}

// The top-left square is black, and the markers fill the white squares,
// from the second.
TEST(Board, TopLeftSquareIsBlack) {
  const core::Result<Board> board = Board::create(legacy_femto_board());
  ASSERT_TRUE(board.ok()) << board.status().message();
  cv::Mat image;  // 150 px per square, (row, column) below
  board->charuco().generateImage(cv::Size(750, 450), image, 0, 1);
  EXPECT_EQ(image.at<unsigned char>(75, 75), 0);    // square (0, 0)
  EXPECT_EQ(image.at<unsigned char>(75, 375), 0);   // square (2, 0)
  EXPECT_EQ(image.at<unsigned char>(225, 225), 0);  // square (1, 1)
  // Square (1, 0) is white around its marker, which is 28.5 px in.
  EXPECT_EQ(image.at<unsigned char>(5, 155), 255);
  EXPECT_EQ(image.at<unsigned char>(145, 295), 255);
}

TEST(Board, NumbersTheMarkersFromTheFirstId) {
  Target target = legacy_femto_board();
  target.first_marker_id = 93;  // the last seven of DICT_4X4_100's codes
  const core::Result<Board> board = Board::create(target);
  ASSERT_TRUE(board.ok()) << board.status().message();
  EXPECT_EQ(board->charuco().getIds(),
            (std::vector<int>{93, 94, 95, 96, 97, 98, 99}));
}

// Each MarkerDictionary is OpenCV's of the same name, and check_target counts
// its codes as OpenCV does.
TEST(Board, EachDictionaryIsOpenCVsOfTheSameName) {
  const std::vector<
      std::pair<MarkerDictionary, cv::aruco::PredefinedDictionaryType>>
      names = {
          {MarkerDictionary::ArUco4x4_50, cv::aruco::DICT_4X4_50},
          {MarkerDictionary::ArUco4x4_100, cv::aruco::DICT_4X4_100},
          {MarkerDictionary::ArUco4x4_250, cv::aruco::DICT_4X4_250},
          {MarkerDictionary::ArUco4x4_1000, cv::aruco::DICT_4X4_1000},
          {MarkerDictionary::ArUco5x5_50, cv::aruco::DICT_5X5_50},
          {MarkerDictionary::ArUco5x5_100, cv::aruco::DICT_5X5_100},
          {MarkerDictionary::ArUco5x5_250, cv::aruco::DICT_5X5_250},
          {MarkerDictionary::ArUco5x5_1000, cv::aruco::DICT_5X5_1000},
          {MarkerDictionary::ArUco6x6_50, cv::aruco::DICT_6X6_50},
          {MarkerDictionary::ArUco6x6_100, cv::aruco::DICT_6X6_100},
          {MarkerDictionary::ArUco6x6_250, cv::aruco::DICT_6X6_250},
          {MarkerDictionary::ArUco6x6_1000, cv::aruco::DICT_6X6_1000},
          {MarkerDictionary::ArUco7x7_50, cv::aruco::DICT_7X7_50},
          {MarkerDictionary::ArUco7x7_100, cv::aruco::DICT_7X7_100},
          {MarkerDictionary::ArUco7x7_250, cv::aruco::DICT_7X7_250},
          {MarkerDictionary::ArUco7x7_1000, cv::aruco::DICT_7X7_1000},
          {MarkerDictionary::ArUcoOriginal, cv::aruco::DICT_ARUCO_ORIGINAL},
          {MarkerDictionary::AprilTag16h5, cv::aruco::DICT_APRILTAG_16h5},
          {MarkerDictionary::AprilTag25h9, cv::aruco::DICT_APRILTAG_25h9},
          {MarkerDictionary::AprilTag36h10, cv::aruco::DICT_APRILTAG_36h10},
          {MarkerDictionary::AprilTag36h11, cv::aruco::DICT_APRILTAG_36h11},
          {MarkerDictionary::ArUcoMip36h12, cv::aruco::DICT_ARUCO_MIP_36h12}};
  ASSERT_EQ(names.size(),
            static_cast<std::size_t>(MarkerDictionary::ArUcoMip36h12));
  for (const auto& [ours, theirs] : names) {
    const cv::aruco::Dictionary expected =
        cv::aruco::getPredefinedDictionary(theirs);
    // One inner corner: 2 x 2 squares, 2 markers, ending at the last code.
    Target target = legacy_femto_board();
    target.rows = 1;
    target.cols = 1;
    target.dictionary = ours;
    target.first_marker_id = expected.bytesList.rows - 2;
    const core::Result<Board> board = Board::create(target);
    ASSERT_TRUE(board.ok())
        << static_cast<int>(ours) << ": " << board.status().message();
    const cv::aruco::Dictionary& got = board->charuco().getDictionary();
    EXPECT_EQ(got.markerSize, expected.markerSize) << static_cast<int>(ours);
    EXPECT_EQ(got.bytesList.size(), expected.bytesList.size())
        << static_cast<int>(ours);
    EXPECT_EQ(cv::norm(got.bytesList, expected.bytesList, cv::NORM_INF), 0.0)
        << static_cast<int>(ours);
    // One code further runs past the dictionary.
    ++target.first_marker_id;
    EXPECT_FALSE(check_target(target).ok()) << static_cast<int>(ours);
  }
}

TEST(Board, RefusesWhatCheckTargetRefuses) {
  Target target = legacy_femto_board();
  target.rows = 0;
  EXPECT_TRUE(RefusedNaming(create_status(target),
                            core::Status::Code::InvalidArgument, "rows"));
  target = legacy_femto_board();
  target.square_size_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(RefusedNaming(create_status(target),
                            core::Status::Code::InvalidArgument,
                            "square_size_m"));
}

TEST(Board, BuildsOnlyAChArUcoBoard) {
  Target grid;
  grid.kind = TargetKind::AprilGrid;
  grid.rows = 4;
  grid.cols = 6;
  grid.square_size_m = 0.05;
  grid.marker_size_m = 0.04;
  grid.dictionary = MarkerDictionary::AprilTag36h11;
  ASSERT_TRUE(check_target(grid).ok());
  EXPECT_TRUE(RefusedNaming(create_status(grid),
                            core::Status::Code::Unsupported, "kind"));
}

// OpenCV keeps the lengths as float: one beyond float, and a marker that
// rounds to the square's size, are refused before they reach it.
TEST(Board, RefusesLengthsAFloatCannotHoldApart) {
  Target huge = legacy_femto_board();
  huge.square_size_m = 1e300;
  EXPECT_TRUE(RefusedNaming(create_status(huge),
                            core::Status::Code::InvalidArgument,
                            "square_size_m"));
  Target close = legacy_femto_board();
  close.marker_size_m = close.square_size_m * (1.0 - 1e-12);
  ASSERT_TRUE(check_target(close).ok());
  EXPECT_TRUE(RefusedNaming(create_status(close),
                            core::Status::Code::InvalidArgument,
                            "marker_size_m"));
}

}  // namespace
}  // namespace volumetric_kit::calib::compute
