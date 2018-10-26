//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "MinimumSpanningTreeTest.h"

#include <iostream>

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(MinimumSpanningTreeTest);

void MinimumSpanningTreeTest::setUp()
{
    // Test Data
    // Reference tree generated using python (scipy.sparse.csgraph.minimum_spanning_tree)
    points_ = { {23, 81},
                {35, 86},
                { 9,  9},
                {14, 95},
                {57, 58},
                {40, 48},
                {19, 61},
                {16, 68},
                {80, 33},
                {40, 51} };
    leaves_ = {1, 2, 3, 8};
    edges_ = { {1, 3, 7},
               {0},
               {5},
               {0},
               {8, 9},
               {2, 9},
               {7, 9},
               {0, 6},
               {4},
               {4, 5, 6} };
    std::unordered_set<Point> point_set(points_.begin(), points_.end());
    tree_ = MinimumSpanningTree(point_set);
}

void MinimumSpanningTreeTest::tearDown()
{
    //Do nothing.
}

void MinimumSpanningTreeTest::testAllPoints() {
    auto verts = tree_.vertices();
    auto points = points_;
    const auto cmp = [](const Point& p1, const Point& p2) { return std::tie(p1.X, p1.Y) < std::tie(p2.X, p2.Y); };
    std::sort(verts.begin(), verts.end(), cmp);
    std::sort(points.begin(), points.end(), cmp);

    CPPUNIT_ASSERT_MESSAGE("Tree does not contain all imput points", verts == points);
}

void MinimumSpanningTreeTest::testLeaves() {
    const auto tree_leaves = tree_.leaves();
    const auto found_all = std::all_of(leaves_.begin(), leaves_.end(),
                                       [&](int idx) {
                                           return std::find(tree_leaves.begin(),
                                                            tree_leaves.end(),
                                                            points_[idx]) != tree_leaves.end(); });
    CPPUNIT_ASSERT_MESSAGE("Tree has incorrect leaves", tree_leaves.size() == leaves_.size() && found_all);
}

void MinimumSpanningTreeTest::testNeighbors() {
    for (auto i = 0; i < edges_.size(); ++i) {
        const auto& neighbors = edges_[i];
        const auto tree_neighbors = tree_.adjacentNodes(points_[i]);
        CPPUNIT_ASSERT(tree_neighbors.size() == neighbors.size());
        CPPUNIT_ASSERT(std::all_of(neighbors.begin(), neighbors.end(),
                                   [&](int idx) {
                                       return std::find(tree_neighbors.begin(),
                                                        tree_neighbors.end(),
                                                        points_[idx]) != tree_neighbors.end(); }));
    }
}

}
