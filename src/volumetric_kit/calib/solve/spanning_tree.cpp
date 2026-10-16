// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/calib/solve/spanning_tree.hpp"

#include <algorithm>
#include <climits>
#include <cstddef>
#include <deque>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
// Not in geometry.hpp, and not self-contained: it needs core.hpp first.
#include <opencv2/geometry/mst.hpp>

#include "volumetric_kit/calib/compute/guarded.hpp"

namespace volumetric_kit::calib::solve {
namespace {

// Refuses a matrix that is not K x K, symmetric and non-negative, each
// message starting with `what`.
core::Status check_covisibility(const Covisibility& shared, std::size_t k,
                                const std::string& what) {
  if (shared.size() != k) {
    return core::Status::invalid_argument(
        what + ": the covisibility has " + std::to_string(shared.size()) +
        " rows for " + std::to_string(k) + " cameras");
  }
  for (std::size_t a = 0; a < k; ++a) {
    if (shared[a].size() != k) {
      return core::Status::invalid_argument(
          what + ": covisibility row " + std::to_string(a) + " has " +
          std::to_string(shared[a].size()) + " counts for " +
          std::to_string(k) + " cameras");
    }
  }
  for (std::size_t a = 0; a < k; ++a) {
    for (std::size_t b = 0; b < k; ++b) {
      if (shared[a][b] < 0 || shared[a][b] != shared[b][a]) {
        return core::Status::invalid_argument(
            what +
            ": the covisibility must be symmetric and non-negative, but [" +
            std::to_string(a) + "][" + std::to_string(b) + "] is " +
            std::to_string(shared[a][b]) + " and [" + std::to_string(b) + "][" +
            std::to_string(a) + "] is " + std::to_string(shared[b][a]));
      }
    }
  }
  return {};
}

core::Status check_reference(std::size_t reference, std::size_t k,
                             const std::string& what) {
  if (reference < k) return {};
  return core::Status::invalid_argument(
      what + ": reference " + std::to_string(reference) +
      " is not one of the rig's " + std::to_string(k) + " cameras");
}

// Which cameras `reference` reaches, breadth first; the inputs are checked.
std::vector<bool> reach(const Covisibility& covisibility,
                        std::size_t reference) {
  const std::size_t k = covisibility.size();
  std::vector<bool> reached(k, false);
  reached[reference] = true;
  std::deque<std::size_t> queue{reference};
  while (!queue.empty()) {
    const std::size_t a = queue.front();
    queue.pop_front();
    for (std::size_t b = 0; b < k; ++b) {
      if (!reached[b] && covisibility[a][b] > 0) {
        reached[b] = true;
        queue.push_back(b);
      }
    }
  }
  return reached;
}

}  // namespace

core::Result<Covisibility> covisibility(
    std::size_t cameras,
    const std::vector<std::vector<std::size_t>>& captures) {
  return compute::guarded("covisibility", [&]() -> core::Result<Covisibility> {
    Covisibility shared(cameras, std::vector<int>(cameras, 0));
    std::vector<bool> listed(cameras, false);
    for (std::size_t k = 0; k < captures.size(); ++k) {
      std::fill(listed.begin(), listed.end(), false);
      for (const std::size_t c : captures[k]) {
        if (c >= cameras) {
          return core::Status::invalid_argument(
              "covisibility: capture " + std::to_string(k) + " names camera " +
              std::to_string(c) + " of a rig of " + std::to_string(cameras));
        }
        if (listed[c]) {
          return core::Status::invalid_argument(
              "covisibility: capture " + std::to_string(k) + " names camera " +
              std::to_string(c) + " twice");
        }
        listed[c] = true;
      }
      for (const std::size_t a : captures[k]) {
        for (const std::size_t b : captures[k]) ++shared[a][b];
      }
    }
    return shared;
  });
}

core::Result<std::vector<bool>> reached_from(const Covisibility& covisibility,
                                             std::size_t reference) {
  return compute::guarded(
      "reached_from", [&]() -> core::Result<std::vector<bool>> {
        const std::size_t k = covisibility.size();
        VKC_TRY(check_covisibility(covisibility, k, "reached_from"));
        VKC_TRY(check_reference(reference, k, "reached_from"));
        return reach(covisibility, reference);
      });
}

core::Result<SpanningTree> spanning_tree(const Covisibility& covisibility,
                                         const std::vector<std::string>& ids,
                                         std::size_t reference,
                                         int weak_below) {
  return compute::guarded("spanning_tree", [&]() -> core::Result<SpanningTree> {
    const std::size_t k = ids.size();
    if (k == 0) {
      return core::Status::invalid_argument(
          "spanning_tree: the rig has no cameras");
    }
    if (k > static_cast<std::size_t>(INT_MAX)) {
      return core::Status::invalid_argument(
          "spanning_tree: more cameras than OpenCV's int counts");
    }
    VKC_TRY(check_covisibility(covisibility, k, "spanning_tree"));
    VKC_TRY(check_reference(reference, k, "spanning_tree"));
    if (weak_below < 0) {
      return core::Status::invalid_argument(
          "spanning_tree: weak_below must not be negative");
    }

    // The cameras in the order of their ids, which breaks every tie.
    std::vector<std::size_t> by_id(k);
    std::iota(by_id.begin(), by_id.end(), std::size_t{0});
    std::stable_sort(
        by_id.begin(), by_id.end(),
        [&](std::size_t a, std::size_t b) { return ids[a] < ids[b]; });
    for (std::size_t r = 1; r < k; ++r) {
      if (ids[by_id[r]] == ids[by_id[r - 1]]) {
        return core::Status::invalid_argument(
            "spanning_tree: camera id \"" + ids[by_id[r]] + "\" appears twice");
      }
    }

    const std::vector<bool> reached = reach(covisibility, reference);
    std::string outside;
    for (const std::size_t c : by_id) {
      if (reached[c]) continue;
      outside += outside.empty() ? "" : ", ";
      outside += ids[c];
    }
    if (!outside.empty()) {
      return core::Status::invalid_argument(
          "spanning_tree: " + outside + " share" +
          (outside.find(", ") == std::string::npos ? "s" : "") +
          " no capture with the tree of the reference, " + ids[reference]);
    }

    SpanningTree tree;
    tree.reference = reference;
    if (k == 1) return tree;

    // Weights: minus the shared captures, plus a fraction below 1 that
    // grows with the pair's place in id order, so no two are equal and the
    // tree is the one maximum tree.
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    for (std::size_t ra = 0; ra < k; ++ra) {
      for (std::size_t rb = ra + 1; rb < k; ++rb) {
        if (covisibility[by_id[ra]][by_id[rb]] > 0) {
          pairs.emplace_back(by_id[ra], by_id[rb]);
        }
      }
    }
    std::vector<cv::MSTEdge> edges;
    edges.reserve(pairs.size());
    for (std::size_t p = 0; p < pairs.size(); ++p) {
      const auto [a, b] = pairs[p];
      edges.push_back(
          {static_cast<int>(a), static_cast<int>(b),
           -static_cast<double>(covisibility[a][b]) +
               static_cast<double>(p) / static_cast<double>(pairs.size())});
    }
    std::vector<cv::MSTEdge> chosen;
    if (!cv::buildMST(static_cast<int>(k), edges, chosen, cv::MST_PRIM,
                      static_cast<int>(reference)) ||
        chosen.size() != k - 1) {
      return core::Status::numerical(
          "spanning_tree: OpenCV's buildMST gave no tree of a connected rig");
    }

    // The links from the reference outwards, siblings in id order.
    std::vector<std::vector<bool>> linked(k, std::vector<bool>(k, false));
    for (const cv::MSTEdge& edge : chosen) {
      const auto a = static_cast<std::size_t>(edge.source);
      const auto b = static_cast<std::size_t>(edge.target);
      if (a >= k || b >= k) {
        return core::Status::numerical(
            "spanning_tree: OpenCV's buildMST gave a link to no camera");
      }
      linked[a][b] = linked[b][a] = true;
    }
    std::vector<bool> placed(k, false);
    placed[reference] = true;
    std::deque<std::size_t> queue{reference};
    while (!queue.empty()) {
      const std::size_t parent = queue.front();
      queue.pop_front();
      for (const std::size_t c : by_id) {
        if (placed[c] || !linked[parent][c]) continue;
        placed[c] = true;
        const int shared = covisibility[c][parent];
        tree.links.push_back({c, parent, shared, shared < weak_below});
        queue.push_back(c);
      }
    }
    if (tree.links.size() != k - 1) {
      return core::Status::numerical(
          "spanning_tree: OpenCV's buildMST gave links that are not a tree");
    }
    return tree;
  });
}

}  // namespace volumetric_kit::calib::solve
