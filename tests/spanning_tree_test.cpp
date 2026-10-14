// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The captures each pair of cameras shares, and the tree that places the
// cameras from the reference through the links that share the most.

#include "volumetric_kit/calib/solve/spanning_tree.hpp"

#include <algorithm>
#include <cstddef>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace volumetric_kit::calib::solve {
namespace {

using Code = core::Status::Code;

core::Result<SpanningTree> tree_of(const Covisibility& shared,
                                   const std::vector<std::string>& ids,
                                   std::size_t reference, int weak_below = 5) {
  core::Result<SpanningTree> tree = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(tree = spanning_tree(shared, ids, reference, weak_below));
  return tree;
}

// Each link as (camera id, parent id).
std::vector<std::pair<std::string, std::string>> named(
    const SpanningTree& tree, const std::vector<std::string>& ids) {
  std::vector<std::pair<std::string, std::string>> links;
  for (const TreeLink& link : tree.links) {
    links.emplace_back(ids[link.camera], ids[link.parent]);
  }
  return links;
}

// The matrix with its cameras reordered: new camera i is old order[i].
Covisibility reordered(const Covisibility& shared,
                       const std::vector<std::size_t>& order) {
  Covisibility out(order.size(), std::vector<int>(order.size()));
  for (std::size_t a = 0; a < order.size(); ++a) {
    for (std::size_t b = 0; b < order.size(); ++b) {
      out[a][b] = shared[order[a]][order[b]];
    }
  }
  return out;
}

TEST(Covisibility, CountsTheCapturesEachPairShares) {
  core::Result<Covisibility> shared = core::Status::invalid_argument("unset");
  EXPECT_NO_THROW(shared = covisibility(4, {{0, 1}, {1, 2, 3}, {}, {2, 1}}));
  ASSERT_TRUE(shared.ok()) << shared.status().message();
  const Covisibility expected = {
      {1, 1, 0, 0}, {1, 3, 2, 1}, {0, 2, 2, 1}, {0, 1, 1, 1}};
  EXPECT_EQ(*shared, expected);
  EXPECT_EQ(covisibility(2, {}).value(), Covisibility(2, {0, 0}));
}

TEST(Covisibility, RefusesACameraOutsideTheRigOrListedTwice) {
  for (const std::vector<std::vector<std::size_t>>& captures :
       std::vector<std::vector<std::vector<std::size_t>>>{{{0, 3}},
                                                          {{0, 1}, {2, 2}}}) {
    core::Result<Covisibility> shared = Covisibility{};
    EXPECT_NO_THROW(shared = covisibility(3, captures));
    EXPECT_EQ(shared.status().domain(), Code::InvalidArgument);
    EXPECT_EQ(shared.status().message().rfind("covisibility: ", 0), 0u);
  }
}

// A rig of four in a ring with one diagonal: the tree takes the links that
// share the most, whatever the reference.
TEST(SpanningTree, TakesTheLinksThatShareTheMost) {
  const std::vector<std::string> ids = {"A", "B", "C", "D"};
  const Covisibility shared = {
      {30, 12, 3, 9}, {12, 30, 7, 0}, {3, 7, 30, 11}, {9, 0, 11, 30}};
  const core::Result<SpanningTree> tree = tree_of(shared, ids, 0);
  ASSERT_TRUE(tree.ok()) << tree.status().message();
  EXPECT_EQ(tree->reference, 0u);
  // Maximum tree: A-B (12), C-D (11), A-D (9); from A, breadth first.
  using Links = std::vector<std::pair<std::string, std::string>>;
  EXPECT_EQ(named(*tree, ids), (Links{{"B", "A"}, {"D", "A"}, {"C", "D"}}));
  EXPECT_EQ(tree->links[0].shared, 12);
  EXPECT_EQ(tree->links[1].shared, 9);
  EXPECT_EQ(tree->links[2].shared, 11);

  const core::Result<SpanningTree> from_c = tree_of(shared, ids, 2);
  ASSERT_TRUE(from_c.ok()) << from_c.status().message();
  EXPECT_EQ(from_c->reference, 2u);
  EXPECT_EQ(named(*from_c, ids), (Links{{"D", "C"}, {"A", "D"}, {"B", "A"}}));
}

// Every link is listed after the one that places its parent, siblings in id
// order, and a link sharing fewer than weak_below captures is weak.
TEST(SpanningTree, ListsLinksOutwardsAndFlagsWeakOnes) {
  const std::vector<std::string> ids = {"Z", "Y", "X", "W", "V"};
  // A star from W, and V hanging off Z.
  const Covisibility shared = {{9, 0, 0, 6, 4},
                               {0, 9, 0, 4, 0},
                               {0, 0, 9, 5, 0},
                               {6, 4, 5, 9, 0},
                               {4, 0, 0, 0, 9}};
  const core::Result<SpanningTree> tree = tree_of(shared, ids, 3);
  ASSERT_TRUE(tree.ok()) << tree.status().message();
  using Links = std::vector<std::pair<std::string, std::string>>;
  EXPECT_EQ(named(*tree, ids),
            (Links{{"X", "W"}, {"Y", "W"}, {"Z", "W"}, {"V", "Z"}}));
  std::map<std::string, bool> weak;
  for (const TreeLink& link : tree->links) weak[ids[link.camera]] = link.weak;
  EXPECT_FALSE(weak["X"]);  // 5 shared
  EXPECT_TRUE(weak["Y"]);   // 4
  EXPECT_FALSE(weak["Z"]);  // 6
  EXPECT_TRUE(weak["V"]);   // 4

  const core::Result<SpanningTree> strict = tree_of(shared, ids, 3, 7);
  ASSERT_TRUE(strict.ok());
  for (const TreeLink& link : strict->links) EXPECT_TRUE(link.weak);
  const core::Result<SpanningTree> lenient = tree_of(shared, ids, 3, 0);
  ASSERT_TRUE(lenient.ok());
  for (const TreeLink& link : lenient->links) EXPECT_FALSE(link.weak);
}

// Links that share as many captures: the pair whose ids sort first wins, so
// the tree is the same for any order of the cameras.
TEST(SpanningTree, BreaksTiesByIdsWhateverTheOrder) {
  const std::vector<std::string> ids = {"c", "a", "d", "b"};
  // Every pair shares 5 captures.
  const Covisibility shared(4, std::vector<int>(4, 5));
  using Links = std::vector<std::pair<std::string, std::string>>;
  std::vector<std::size_t> order = {0, 1, 2, 3};
  do {
    std::vector<std::string> permuted;
    for (const std::size_t i : order) permuted.push_back(ids[i]);
    const std::size_t reference =
        std::find(permuted.begin(), permuted.end(), "d") - permuted.begin();
    const core::Result<SpanningTree> tree =
        tree_of(reordered(shared, order), permuted, reference);
    ASSERT_TRUE(tree.ok()) << tree.status().message();
    // Pairs in id order: a-b, a-c, a-d, b-c, ...: the maximum tree is the
    // star at a, reached from d through a-d.
    EXPECT_EQ(named(*tree, permuted),
              (Links{{"a", "d"}, {"b", "a"}, {"c", "a"}}));
  } while (std::next_permutation(order.begin(), order.end()));
}

TEST(SpanningTree, NamesEveryCameraOutsideTheReferencesTree) {
  const std::vector<std::string> ids = {"SN-A", "SN-B", "SN-C", "SN-D"};
  // A-B and C-D, apart.
  const Covisibility shared = {
      {4, 3, 0, 0}, {3, 4, 0, 0}, {0, 0, 4, 2}, {0, 0, 2, 4}};
  const core::Result<SpanningTree> tree = tree_of(shared, ids, 1);
  EXPECT_EQ(tree.status().domain(), Code::InvalidArgument);
  EXPECT_EQ(tree.status().message(),
            "spanning_tree: SN-C, SN-D share no capture with the tree of the "
            "reference, SN-B");

  Covisibility one_apart = shared;
  one_apart[1][2] = one_apart[2][1] = 1;
  const core::Result<SpanningTree> lone = tree_of(one_apart, ids, 0);
  ASSERT_TRUE(lone.ok()) << lone.status().message();
  one_apart[2][3] = one_apart[3][2] = 0;
  EXPECT_EQ(tree_of(one_apart, ids, 0).status().message(),
            "spanning_tree: SN-D shares no capture with the tree of the "
            "reference, SN-A");
}

TEST(SpanningTree, ARigOfOneIsItsReference) {
  const core::Result<SpanningTree> tree = tree_of({{3}}, {"only"}, 0);
  ASSERT_TRUE(tree.ok()) << tree.status().message();
  EXPECT_EQ(tree->reference, 0u);
  EXPECT_TRUE(tree->links.empty());
}

TEST(SpanningTree, RefusesInputsThatDoNotAgree) {
  const std::vector<std::string> ids = {"A", "B"};
  const Covisibility good = {{2, 1}, {1, 2}};
  const struct {
    const char* name;
    Covisibility shared;
    std::vector<std::string> ids;
    std::size_t reference;
    int weak_below;
  } cases[] = {
      {"no cameras", {}, {}, 0, 5},
      {"too few rows", {{2, 1}}, ids, 0, 5},
      {"a short row", {{2, 1}, {1}}, ids, 0, 5},
      {"not symmetric", {{2, 1}, {0, 2}}, ids, 0, 5},
      {"a negative count", {{2, -1}, {-1, 2}}, ids, 0, 5},
      {"a repeated id", good, {"A", "A"}, 0, 5},
      {"a reference outside", good, ids, 2, 5},
      {"a negative weak_below", good, ids, 0, -1},
  };
  for (const auto& c : cases) {
    const core::Result<SpanningTree> tree =
        tree_of(c.shared, c.ids, c.reference, c.weak_below);
    EXPECT_EQ(tree.status().domain(), Code::InvalidArgument) << c.name;
    EXPECT_EQ(tree.status().message().rfind("spanning_tree: ", 0), 0u)
        << c.name << ": " << tree.status().message();
  }
}

}  // namespace
}  // namespace volumetric_kit::calib::solve
