/**
 * @file test_spatial_grid.cpp
 * @brief Unit tests for the internal SpatialGrid neighbour index.
 */

#include "../../src/internal/spatial_grid.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

namespace bp3d::internal::test {

namespace {

AABB box(double x, double y, double z, double w, double h, double d) {
    return AABB{{x, y, z}, {x + w, y + h, z + d}};
}

bool contains(const std::vector<std::size_t>& v, std::size_t value) {
    return std::find(v.begin(), v.end(), value) != v.end();
}

}  // namespace

class SpatialGridTest : public ::testing::Test {
protected:
    SpatialGrid grid{10.0};
    std::vector<std::size_t> out;
};

TEST_F(SpatialGridTest, EmptyGridReturnsNothing) {
    grid.query(box(0, 0, 0, 100, 100, 100), out);
    EXPECT_TRUE(out.empty());
    EXPECT_EQ(grid.size(), 0U);
}

TEST_F(SpatialGridTest, NonPositiveCellSizeIsClamped) {
    SpatialGrid degenerate{0.0};
    EXPECT_GT(degenerate.cell_size(), 0.0);
    degenerate.insert(0, box(0, 0, 0, 1, 1, 1));
    std::vector<std::size_t> result;
    degenerate.query(box(0, 0, 0, 1, 1, 1), result);
    EXPECT_TRUE(contains(result, 0U));
}

TEST_F(SpatialGridTest, FindsOverlappingBox) {
    grid.insert(0, box(0, 0, 0, 10, 10, 10));
    grid.query(box(5, 5, 5, 10, 10, 10), out);
    EXPECT_TRUE(contains(out, 0U));
    EXPECT_EQ(grid.size(), 1U);
}

TEST_F(SpatialGridTest, IgnoresDistantBox) {
    grid.insert(0, box(0, 0, 0, 5, 5, 5));
    grid.query(box(500, 500, 500, 5, 5, 5), out);
    EXPECT_FALSE(contains(out, 0U));
}

TEST_F(SpatialGridTest, ReturnsEachIndexAtMostOnce) {
    // A box spanning many cells, queried by another large box, must appear once.
    grid.insert(7, box(0, 0, 0, 100, 100, 100));
    grid.query(box(0, 0, 0, 100, 100, 100), out);
    EXPECT_EQ(std::count(out.begin(), out.end(), 7U), 1);
}

TEST_F(SpatialGridTest, QueryAppendsWithoutClearing) {
    out.push_back(999U);  // pre-existing content must be preserved
    grid.insert(0, box(0, 0, 0, 10, 10, 10));
    grid.query(box(0, 0, 0, 10, 10, 10), out);
    EXPECT_TRUE(contains(out, 999U));
    EXPECT_TRUE(contains(out, 0U));
}

TEST_F(SpatialGridTest, FindsTouchingBoxBelow) {
    // A supporter whose top face exactly meets the query box's bottom must be
    // reported — this underpins the support / stacking checks.
    grid.insert(0, box(0, 0, 0, 10, 10, 10));    // top at y = 10
    grid.query(box(0, 10, 0, 10, 10, 10), out);  // sits exactly on top
    EXPECT_TRUE(contains(out, 0U));
}

TEST_F(SpatialGridTest, ReturnsSupersetOfAllOverlaps) {
    // Property test: every box that truly overlaps the query must be returned.
    std::vector<AABB> boxes;
    for (int i = 0; i < 50; ++i) {
        const double base = static_cast<double>(i) * 7.0;
        boxes.push_back(box(base, base * 0.5, base * 0.25, 12, 9, 6));
        grid.insert(static_cast<std::size_t>(i), boxes.back());
    }

    const AABB query = box(30, 20, 10, 40, 30, 20);
    grid.query(query, out);

    for (std::size_t i = 0; i < boxes.size(); ++i) {
        const bool overlaps = query.overlaps(boxes[i]);
        if (overlaps) {
            EXPECT_TRUE(contains(out, i)) << "missed overlapping box " << i;
        }
    }
}

TEST_F(SpatialGridTest, ClearRemovesAllBoxes) {
    grid.insert(0, box(0, 0, 0, 10, 10, 10));
    grid.insert(1, box(20, 0, 0, 10, 10, 10));
    grid.clear();
    EXPECT_EQ(grid.size(), 0U);

    grid.query(box(0, 0, 0, 10, 10, 10), out);
    EXPECT_TRUE(out.empty());
}

TEST_F(SpatialGridTest, ReusableAcrossManyQueries) {
    // Exercise epoch stamping across repeated queries (dedup must stay correct).
    grid.insert(0, box(0, 0, 0, 10, 10, 10));
    for (int i = 0; i < 1000; ++i) {
        out.clear();
        grid.query(box(0, 0, 0, 10, 10, 10), out);
        EXPECT_EQ(std::count(out.begin(), out.end(), 0U), 1);
    }
}

}  // namespace bp3d::internal::test
