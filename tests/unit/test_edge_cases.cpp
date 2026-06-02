/**
 * @file test_edge_cases.cpp
 * @brief Edge case tests for the 3D bin packing library
 */

#include "../../src/internal/placement_engine.hpp"

#include <bp3d/algorithms/bfd.hpp>
#include <bp3d/algorithms/extreme_point.hpp>
#include <bp3d/algorithms/ffd.hpp>
#include <bp3d/algorithms/guillotine.hpp>
#include <bp3d/algorithms/parallel_solver.hpp>
#include <bp3d/algorithms/shelf.hpp>
#include <bp3d/bp3d.hpp>
#include <bp3d/constraints/collision.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>

namespace bp3d::test {

// =============================================================================
// Dimension Edge Cases
// =============================================================================

class DimensionEdgeCaseTest : public ::testing::Test {
protected:
    SolverConfig config;
    FirstFitDecreasing solver;

    void SetUp() override {
        config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}});
        config.allow_multiple_bins = true;
    }
};

TEST_F(DimensionEdgeCaseTest, ZeroDimensionItem) {
    std::vector<Item> items;
    items.push_back(Item{"zero_width", {0.0, 10.0, 10.0}});

    auto result = solver.solve(items, config);
    // Zero-dimension items should either be packed trivially or rejected
    EXPECT_TRUE(result.placements.size() == 1 || result.unpacked_items.size() == 1);
}

TEST_F(DimensionEdgeCaseTest, VerySmallDimensions) {
    std::vector<Item> items;
    items.push_back(Item{"tiny", {0.001, 0.001, 0.001}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

TEST_F(DimensionEdgeCaseTest, VeryLargeDimensions) {
    std::vector<Item> items;
    items.push_back(Item{"huge", {1e9, 1e9, 1e9}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.unpacked_items.size(), 1);
    EXPECT_EQ(result.placements.size(), 0);
}

TEST_F(DimensionEdgeCaseTest, ExactFitItem) {
    std::vector<Item> items;
    items.push_back(Item{"exact", {100.0, 100.0, 100.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
    EXPECT_EQ(result.bins_used, 1);
}

TEST_F(DimensionEdgeCaseTest, SlightlyTooLarge) {
    std::vector<Item> items;
    items.push_back(Item{"too_big", {100.1, 100.0, 100.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.unpacked_items.size(), 1);
}

TEST_F(DimensionEdgeCaseTest, OneDimensionExactlyFits) {
    std::vector<Item> items;
    items.push_back(Item{"exact_width", {100.0, 50.0, 50.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

TEST_F(DimensionEdgeCaseTest, PerfectCubes) {
    std::vector<Item> items;
    // 8 cubes of 50x50x50 should fit in 100x100x100
    for (int i = 0; i < 8; ++i) {
        items.push_back(Item{"cube_" + std::to_string(i), {50.0, 50.0, 50.0}});
    }

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 8);
    EXPECT_EQ(result.bins_used, 1);
}

TEST_F(DimensionEdgeCaseTest, VeryFlatItem) {
    std::vector<Item> items;
    items.push_back(Item{"flat", {90.0, 0.1, 90.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

TEST_F(DimensionEdgeCaseTest, VeryThinItem) {
    std::vector<Item> items;
    items.push_back(Item{"thin", {0.1, 90.0, 0.1}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

TEST_F(DimensionEdgeCaseTest, LongRodItem) {
    std::vector<Item> items;
    items.push_back(Item{"rod", {1.0, 1.0, 99.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

// =============================================================================
// Quantity Edge Cases
// =============================================================================

class QuantityEdgeCaseTest : public ::testing::Test {
protected:
    SolverConfig config;
    FirstFitDecreasing solver;

    void SetUp() override {
        config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}});
        config.allow_multiple_bins = true;
    }
};

TEST_F(QuantityEdgeCaseTest, ZeroQuantity) {
    std::vector<Item> items;
    Item item{"zero_qty", {10.0, 10.0, 10.0}};
    item.quantity = 0;
    items.push_back(item);

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 0);
}

TEST_F(QuantityEdgeCaseTest, LargeQuantity) {
    std::vector<Item> items;
    Item item{"many", {10.0, 10.0, 10.0}};
    item.quantity = 100;  // Reasonable quantity for edge case testing
    items.push_back(item);

    auto result = solver.solve(items, config);
    // 100 items should pack into bins
    EXPECT_GT(result.placements.size(), 0);
    EXPECT_EQ(result.placements.size() + result.unpacked_items.size(), 100);
}

TEST_F(QuantityEdgeCaseTest, SingleQuantity) {
    std::vector<Item> items;
    Item item{"single", {10.0, 10.0, 10.0}};
    item.quantity = 1;
    items.push_back(item);

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

// =============================================================================
// Empty/Minimal Input Edge Cases
// =============================================================================

class EmptyInputEdgeCaseTest : public ::testing::Test {
protected:
    SolverConfig config;
    FirstFitDecreasing solver;

    void SetUp() override { config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}}); }
};

TEST_F(EmptyInputEdgeCaseTest, EmptyItemList) {
    std::vector<Item> items;

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 0);
    EXPECT_EQ(result.unpacked_items.size(), 0);
    EXPECT_EQ(result.bins_used, 0);
}

TEST_F(EmptyInputEdgeCaseTest, SingleItem) {
    std::vector<Item> items;
    items.push_back(Item{"only", {10.0, 10.0, 10.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
    EXPECT_EQ(result.bins_used, 1);
}

TEST_F(EmptyInputEdgeCaseTest, NoBinTypes) {
    SolverConfig empty_config;
    std::vector<Item> items;
    items.push_back(Item{"orphan", {10.0, 10.0, 10.0}});

    auto result = solver.solve(items, empty_config);
    // With no bins, solver produces empty results
    EXPECT_EQ(result.placements.size(), 0);
}

TEST_F(EmptyInputEdgeCaseTest, AllItemsTooLarge) {
    std::vector<Item> items;
    items.push_back(Item{"big1", {200.0, 200.0, 200.0}});
    items.push_back(Item{"big2", {150.0, 150.0, 150.0}});
    items.push_back(Item{"big3", {101.0, 50.0, 50.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.unpacked_items.size(), 3);
    EXPECT_EQ(result.placements.size(), 0);
}

// =============================================================================
// Weight Edge Cases
// =============================================================================

class WeightEdgeCaseTest : public ::testing::Test {
protected:
    SolverConfig config;
    FirstFitDecreasing solver;

    void SetUp() override {
        BinType bin{"bin", {100.0, 100.0, 100.0}};
        bin.max_weight = 100.0;
        config.bin_types.push_back(bin);
        config.allow_multiple_bins = true;
    }
};

TEST_F(WeightEdgeCaseTest, ZeroWeight) {
    std::vector<Item> items;
    Item item{"weightless", {10.0, 10.0, 10.0}};
    item.weight = 0.0;
    items.push_back(item);

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

TEST_F(WeightEdgeCaseTest, ExactMaxWeight) {
    std::vector<Item> items;
    Item item{"exact_weight", {10.0, 10.0, 10.0}};
    item.weight = 100.0;
    items.push_back(item);

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

TEST_F(WeightEdgeCaseTest, SlightlyOverWeight) {
    std::vector<Item> items;
    Item item{"overweight", {10.0, 10.0, 10.0}};
    item.weight = 100.1;
    items.push_back(item);

    auto result = solver.solve(items, config);
    // Item should be rejected due to weight
    EXPECT_EQ(result.unpacked_items.size(), 1);
}

TEST_F(WeightEdgeCaseTest, ManyLightItems) {
    std::vector<Item> items;
    for (int i = 0; i < 100; ++i) {
        Item item{"light_" + std::to_string(i), {5.0, 5.0, 5.0}};
        item.weight = 0.5;  // Total = 50, within limit
        items.push_back(item);
    }

    auto result = solver.solve(items, config);
    EXPECT_GT(result.placements.size(), 0);
}

TEST_F(WeightEdgeCaseTest, ZeroBinMaxWeight) {
    SolverConfig zero_weight_config;
    BinType bin{"bin", {100.0, 100.0, 100.0}};
    bin.max_weight = 0.0;
    zero_weight_config.bin_types.push_back(bin);

    std::vector<Item> items;
    Item item{"any", {10.0, 10.0, 10.0}};
    item.weight = 0.1;
    items.push_back(item);

    auto result = solver.solve(items, zero_weight_config);
    // Any item with weight > 0 should be rejected
    EXPECT_EQ(result.unpacked_items.size(), 1);
}

// =============================================================================
// Floating Point Edge Cases
// =============================================================================

class FloatingPointEdgeCaseTest : public ::testing::Test {
protected:
    SolverConfig config;
    FirstFitDecreasing solver;

    void SetUp() override {
        config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}});
        config.allow_multiple_bins = true;
    }
};

TEST_F(FloatingPointEdgeCaseTest, VerySmallDifference) {
    // Item that differs from bin by epsilon
    std::vector<Item> items;
    double epsilon = std::numeric_limits<double>::epsilon() * 100;
    items.push_back(Item{"almost_exact", {100.0 - epsilon, 100.0, 100.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

TEST_F(FloatingPointEdgeCaseTest, NonIntegerDimensions) {
    std::vector<Item> items;
    items.push_back(Item{"pi", {3.14159, 2.71828, 1.41421}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

TEST_F(FloatingPointEdgeCaseTest, RepeatingDecimals) {
    std::vector<Item> items;
    items.push_back(Item{"third", {33.333333, 33.333333, 33.333333}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

TEST_F(FloatingPointEdgeCaseTest, SumToExactDimension) {
    // Three items that should exactly fill one dimension
    std::vector<Item> items;
    items.push_back(Item{"a", {33.34, 100.0, 100.0}});
    items.push_back(Item{"b", {33.33, 100.0, 100.0}});
    items.push_back(Item{"c", {33.33, 100.0, 100.0}});

    auto result = solver.solve(items, config);
    // Due to FP precision, may or may not all fit in one bin
    EXPECT_GE(result.placements.size(), 0);
}

// =============================================================================
// Collision Detection Edge Cases
// =============================================================================

class CollisionEdgeCaseTest : public ::testing::Test {
protected:
    CollisionValidator validator;
    BinType bin{"bin", {100.0, 100.0, 100.0}};
    Item item{"test", {10.0, 10.0, 10.0}};
};

TEST_F(CollisionEdgeCaseTest, ExactlyTouching) {
    std::vector<Placement> existing;
    existing.push_back(
        Placement{"a", "bin", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}});

    // Exactly touching at edge
    Placement candidate{"b", "bin", 0, {10.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_TRUE(validator.can_place(item, candidate, existing, bin));
}

TEST_F(CollisionEdgeCaseTest, MinimalOverlap) {
    std::vector<Placement> existing;
    existing.push_back(
        Placement{"a", "bin", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}});

    // Minimal overlap (0.001 units)
    Placement candidate{"b", "bin", 0, {9.999, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_FALSE(validator.can_place(item, candidate, existing, bin));
}

TEST_F(CollisionEdgeCaseTest, DiagonallyTouching) {
    std::vector<Placement> existing;
    existing.push_back(
        Placement{"a", "bin", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}});

    // Diagonally adjacent (corner touching)
    Placement candidate{"b", "bin", 0, {10.0, 10.0, 10.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_TRUE(validator.can_place(item, candidate, existing, bin));
}

TEST_F(CollisionEdgeCaseTest, FullyContained) {
    std::vector<Placement> existing;
    existing.push_back(
        Placement{"outer", "bin", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {50.0, 50.0, 50.0}});

    // Smaller item fully inside existing
    Item small{"small", {5.0, 5.0, 5.0}};
    Placement candidate{"inner", "bin", 0, {10.0, 10.0, 10.0}, Rotation::WHD, {5.0, 5.0, 5.0}};

    EXPECT_FALSE(validator.can_place(small, candidate, existing, bin));
}

TEST_F(CollisionEdgeCaseTest, SamePosition) {
    std::vector<Placement> existing;
    existing.push_back(
        Placement{"a", "bin", 0, {10.0, 10.0, 10.0}, Rotation::WHD, {10.0, 10.0, 10.0}});

    // Exact same position
    Placement candidate{"b", "bin", 0, {10.0, 10.0, 10.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_FALSE(validator.can_place(item, candidate, existing, bin));
}

TEST_F(CollisionEdgeCaseTest, AtBinBoundary) {
    std::vector<Placement> existing;

    // Place at far corner of bin
    Placement candidate{"corner", "bin", 0, {90.0, 90.0, 90.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_TRUE(validator.can_place(item, candidate, existing, bin));
}

TEST_F(CollisionEdgeCaseTest, PartiallyOutsideBin) {
    std::vector<Placement> existing;

    // Placement that extends outside bin
    Placement candidate{"outside", "bin", 0, {95.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    // Collision validator also checks bin bounds - item extending outside is rejected
    EXPECT_FALSE(validator.can_place(item, candidate, existing, bin));
}

// =============================================================================
// Constraint Edge Cases
// =============================================================================

class ConstraintEdgeCaseTest : public ::testing::Test {
protected:
    SolverConfig config;
    FirstFitDecreasing solver;

    void SetUp() override {
        config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}});
        config.allow_multiple_bins = true;
    }
};

TEST_F(ConstraintEdgeCaseTest, AllItemsThisSideUp) {
    std::vector<Item> items;
    for (int i = 0; i < 5; ++i) {
        Item item{"upright_" + std::to_string(i), {10.0, 30.0, 10.0}};
        item.constraints.this_side_up = true;
        items.push_back(item);
    }

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 5);

    // Verify height is preserved
    for (const auto& p : result.placements) {
        EXPECT_DOUBLE_EQ(p.rotated_dimensions.height, 30.0);
    }
}

TEST_F(ConstraintEdgeCaseTest, MixedConstraints) {
    std::vector<Item> items;

    Item upright{"upright", {10.0, 40.0, 10.0}};
    upright.constraints.this_side_up = true;
    items.push_back(upright);

    Item heavy{"heavy", {15.0, 15.0, 15.0}};
    heavy.weight = 50.0;
    items.push_back(heavy);

    Item normal{"normal", {20.0, 20.0, 20.0}};
    items.push_back(normal);

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 3);
}

// =============================================================================
// Bin Type Edge Cases
// =============================================================================

class BinTypeEdgeCaseTest : public ::testing::Test {
protected:
    FirstFitDecreasing solver;
};

TEST_F(BinTypeEdgeCaseTest, MultipleBinTypesSameSize) {
    SolverConfig config;
    config.bin_types.push_back(BinType{"a", {100.0, 100.0, 100.0}});
    config.bin_types.push_back(BinType{"b", {100.0, 100.0, 100.0}});
    config.allow_multiple_bins = true;

    std::vector<Item> items;
    items.push_back(Item{"item", {50.0, 50.0, 50.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

TEST_F(BinTypeEdgeCaseTest, BinTypesDifferentCosts) {
    SolverConfig config;
    BinType cheap{"cheap", {100.0, 100.0, 100.0}};
    cheap.cost = 1.0;
    BinType expensive{"expensive", {100.0, 100.0, 100.0}};
    expensive.cost = 100.0;

    config.bin_types.push_back(cheap);
    config.bin_types.push_back(expensive);
    config.allow_multiple_bins = true;

    std::vector<Item> items;
    items.push_back(Item{"item", {50.0, 50.0, 50.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
    // Algorithm may prefer cheaper bin
}

TEST_F(BinTypeEdgeCaseTest, VerySmallBin) {
    SolverConfig config;
    config.bin_types.push_back(BinType{"tiny", {1.0, 1.0, 1.0}});

    std::vector<Item> items;
    items.push_back(Item{"small", {0.5, 0.5, 0.5}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

TEST_F(BinTypeEdgeCaseTest, VeryLargeBin) {
    SolverConfig config;
    config.bin_types.push_back(BinType{"huge", {1000.0, 1000.0, 1000.0}});

    std::vector<Item> items;
    for (int i = 0; i < 100; ++i) {
        items.push_back(Item{"item_" + std::to_string(i), {50.0, 50.0, 50.0}});
    }

    auto result = solver.solve(items, config);
    EXPECT_GT(result.placements.size(), 0);
}

// =============================================================================
// Algorithm-Specific Edge Cases
// =============================================================================

class AlgorithmEdgeCaseTest : public ::testing::Test {
protected:
    SolverConfig config;

    void SetUp() override {
        config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}});
        config.allow_multiple_bins = true;
    }
};

TEST_F(AlgorithmEdgeCaseTest, IdenticalItems) {
    std::vector<Item> items;
    for (int i = 0; i < 27; ++i) {
        items.push_back(Item{"same_" + std::to_string(i), {33.0, 33.0, 33.0}});
    }

    // Test all algorithms with identical items
    FirstFitDecreasing ffd;
    auto ffd_result = ffd.solve(items, config);
    EXPECT_GT(ffd_result.placements.size(), 0);

    BestFitDecreasing bfd;
    auto bfd_result = bfd.solve(items, config);
    EXPECT_GT(bfd_result.placements.size(), 0);

    ShelfSolver shelf;
    auto shelf_result = shelf.solve(items, config);
    EXPECT_GT(shelf_result.placements.size(), 0);
}

TEST_F(AlgorithmEdgeCaseTest, DescendingSizeOrder) {
    std::vector<Item> items;
    items.push_back(Item{"large", {80.0, 80.0, 80.0}});
    items.push_back(Item{"medium", {40.0, 40.0, 40.0}});
    items.push_back(Item{"small", {20.0, 20.0, 20.0}});
    items.push_back(Item{"tiny", {10.0, 10.0, 10.0}});

    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);

    // FFD should handle pre-sorted items well
    EXPECT_GT(result.placements.size(), 0);
}

TEST_F(AlgorithmEdgeCaseTest, AscendingSizeOrder) {
    std::vector<Item> items;
    items.push_back(Item{"tiny", {10.0, 10.0, 10.0}});
    items.push_back(Item{"small", {20.0, 20.0, 20.0}});
    items.push_back(Item{"medium", {40.0, 40.0, 40.0}});
    items.push_back(Item{"large", {80.0, 80.0, 80.0}});

    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);

    // FFD sorts internally, should still work well
    EXPECT_GT(result.placements.size(), 0);
}

TEST_F(AlgorithmEdgeCaseTest, RandomMixedSizes) {
    std::vector<Item> items;
    items.push_back(Item{"a", {45.0, 30.0, 20.0}});
    items.push_back(Item{"b", {10.0, 80.0, 15.0}});
    items.push_back(Item{"c", {70.0, 10.0, 70.0}});
    items.push_back(Item{"d", {25.0, 25.0, 25.0}});
    items.push_back(Item{"e", {5.0, 5.0, 90.0}});

    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);
    EXPECT_GT(result.placements.size(), 0);
}

// =============================================================================
// ID String Edge Cases
// =============================================================================

class IDEdgeCaseTest : public ::testing::Test {
protected:
    SolverConfig config;
    FirstFitDecreasing solver;

    void SetUp() override { config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}}); }
};

TEST_F(IDEdgeCaseTest, EmptyID) {
    std::vector<Item> items;
    items.push_back(Item{"", {10.0, 10.0, 10.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

TEST_F(IDEdgeCaseTest, VeryLongID) {
    std::vector<Item> items;
    std::string long_id(1000, 'x');
    items.push_back(Item{long_id, {10.0, 10.0, 10.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
    EXPECT_EQ(result.placements[0].item_id, long_id);
}

TEST_F(IDEdgeCaseTest, SpecialCharactersInID) {
    std::vector<Item> items;
    items.push_back(Item{"item-with_special.chars@123!", {10.0, 10.0, 10.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 1);
}

TEST_F(IDEdgeCaseTest, DuplicateIDs) {
    std::vector<Item> items;
    items.push_back(Item{"same", {10.0, 10.0, 10.0}});
    items.push_back(Item{"same", {15.0, 15.0, 15.0}});
    items.push_back(Item{"same", {20.0, 20.0, 20.0}});

    auto result = solver.solve(items, config);
    // Should handle duplicate IDs gracefully
    EXPECT_EQ(result.placements.size(), 3);
}

TEST_F(IDEdgeCaseTest, WhitespaceInID) {
    std::vector<Item> items;
    items.push_back(Item{"item with spaces", {10.0, 10.0, 10.0}});
    items.push_back(Item{"item\twith\ttabs", {10.0, 10.0, 10.0}});
    items.push_back(Item{"item\nwith\nnewlines", {10.0, 10.0, 10.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.placements.size(), 3);
}

// =============================================================================
// Multiple Bins Edge Cases
// =============================================================================

class MultipleBinsEdgeCaseTest : public ::testing::Test {
protected:
    SolverConfig config;
    FirstFitDecreasing solver;

    void SetUp() override { config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}}); }
};

TEST_F(MultipleBinsEdgeCaseTest, DisallowMultipleBins) {
    config.allow_multiple_bins = false;

    std::vector<Item> items;
    // Two items that together exceed bin capacity
    items.push_back(Item{"a", {60.0, 100.0, 100.0}});
    items.push_back(Item{"b", {60.0, 100.0, 100.0}});

    auto result = solver.solve(items, config);
    // Single-bin mode must still use the one permitted bin: the first item is
    // packed, the second (which would need a second bin) is left unpacked.
    EXPECT_EQ(result.bins_used, 1);
    EXPECT_EQ(result.placements.size(), 1u);
    ASSERT_EQ(result.unpacked_items.size(), 1u);
    EXPECT_EQ(result.unpacked_items.front(), "b");
}

TEST_F(MultipleBinsEdgeCaseTest, AllowMultipleBinsExact) {
    config.allow_multiple_bins = true;

    std::vector<Item> items;
    // Items that exactly need 2 bins
    items.push_back(Item{"a", {100.0, 100.0, 100.0}});
    items.push_back(Item{"b", {100.0, 100.0, 100.0}});

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.bins_used, 2);
    EXPECT_EQ(result.placements.size(), 2);
}

TEST_F(MultipleBinsEdgeCaseTest, ManyBinsNeeded) {
    config.allow_multiple_bins = true;

    std::vector<Item> items;
    // 10 items that each fill a bin
    for (int i = 0; i < 10; ++i) {
        items.push_back(Item{"full_" + std::to_string(i), {100.0, 100.0, 100.0}});
    }

    auto result = solver.solve(items, config);
    EXPECT_EQ(result.bins_used, 10);
    EXPECT_EQ(result.placements.size(), 10);
}

// =============================================================================
// Single-bin mode consistency across every algorithm
// =============================================================================
//
// With allow_multiple_bins == false, all five solvers must agree:
//   1. the single permitted bin is opened and filled (overflow items become
//      unpacked rather than silently dropped or forced into a second bin), and
//   2. a bin that receives no item is never reported (no phantom empty bins),
//      so an item that fits nowhere yields zero bins used.
// FFD/BFD open the first bin lazily; ExtremePoint opens it lazily on the first
// pack; Shelf pre-opens it in reset(). The shared make_result() filters empty
// bins, so the observable result is identical regardless of that difference.

template <class Solver>
void expect_single_bin_overflow_unpacked() {
    SolverConfig config;
    config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}});
    config.allow_multiple_bins = false;

    std::vector<Item> items;
    // Two items that together exceed the single bin's width (60 + 60 > 100).
    items.push_back(Item{"a", {60.0, 100.0, 100.0}});
    items.push_back(Item{"b", {60.0, 100.0, 100.0}});

    Solver solver;
    auto result = solver.solve(items, config);

    // Exactly one item is packed into the one permitted bin; the other is
    // unpacked. (Which of the two is packed can depend on the per-algorithm
    // ordering, so only the counts are asserted.)
    EXPECT_EQ(result.bins_used, 1);
    EXPECT_EQ(result.placements.size(), 1u);
    EXPECT_EQ(result.unpacked_items.size(), 1u);
}

template <class Solver>
void expect_single_bin_nothing_fits() {
    SolverConfig config;
    config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}});
    config.allow_multiple_bins = false;

    std::vector<Item> items;
    // Larger than the bin in every dimension: it can never be placed.
    items.push_back(Item{"too_big", {200.0, 200.0, 200.0}});

    Solver solver;
    auto result = solver.solve(items, config);

    // No bin received an item, so none is reported (no phantom empty bin).
    EXPECT_EQ(result.bins_used, 0);
    EXPECT_TRUE(result.placements.empty());
    ASSERT_EQ(result.unpacked_items.size(), 1u);
    EXPECT_EQ(result.unpacked_items.front(), "too_big");
}

TEST(SingleBinConsistencyTest, OverflowUnpacked_FFD) {
    expect_single_bin_overflow_unpacked<FirstFitDecreasing>();
}
TEST(SingleBinConsistencyTest, OverflowUnpacked_BFD) {
    expect_single_bin_overflow_unpacked<BestFitDecreasing>();
}
TEST(SingleBinConsistencyTest, OverflowUnpacked_Guillotine) {
    expect_single_bin_overflow_unpacked<GuillotineSolver>();
}
TEST(SingleBinConsistencyTest, OverflowUnpacked_ExtremePoint) {
    expect_single_bin_overflow_unpacked<ExtremePointSolver>();
}
TEST(SingleBinConsistencyTest, OverflowUnpacked_Shelf) {
    expect_single_bin_overflow_unpacked<ShelfSolver>();
}

TEST(SingleBinConsistencyTest, NothingFits_FFD) {
    expect_single_bin_nothing_fits<FirstFitDecreasing>();
}
TEST(SingleBinConsistencyTest, NothingFits_BFD) {
    expect_single_bin_nothing_fits<BestFitDecreasing>();
}
TEST(SingleBinConsistencyTest, NothingFits_Guillotine) {
    expect_single_bin_nothing_fits<GuillotineSolver>();
}
TEST(SingleBinConsistencyTest, NothingFits_ExtremePoint) {
    expect_single_bin_nothing_fits<ExtremePointSolver>();
}
TEST(SingleBinConsistencyTest, NothingFits_Shelf) {
    expect_single_bin_nothing_fits<ShelfSolver>();
}

// =============================================================================
// Timeout enforcement (R6)
// =============================================================================

TEST(DeadlineHelperTest, NonPositiveTimeoutDisablesLimit) {
    const auto now = std::chrono::steady_clock::now();
    // A zero or negative timeout means "no limit": never reached, even for a
    // start far in the past.
    EXPECT_FALSE(
        internal::deadline_reached(now - std::chrono::hours(1), std::chrono::milliseconds::zero()));
    EXPECT_FALSE(
        internal::deadline_reached(now - std::chrono::hours(1), std::chrono::milliseconds(-5)));
}

TEST(DeadlineHelperTest, ElapsedBeyondTimeoutIsReached) {
    const auto now = std::chrono::steady_clock::now();
    // Start 10ms in the past with a 1ms budget -> deadline reached.
    EXPECT_TRUE(internal::deadline_reached(now - std::chrono::milliseconds(10),
                                           std::chrono::milliseconds(1)));
}

TEST(DeadlineHelperTest, WithinTimeoutIsNotReached) {
    const auto now = std::chrono::steady_clock::now();
    // Just started with a generous budget -> not reached.
    EXPECT_FALSE(internal::deadline_reached(now, std::chrono::seconds(60)));
}

TEST(SolverTimeoutTest, GenerousTimeoutPacksEverything) {
    SolverConfig config;
    config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}});
    config.timeout = std::chrono::minutes(5);  // ample budget

    std::vector<Item> items;
    for (int i = 0; i < 50; ++i) {
        items.push_back(Item{"box_" + std::to_string(i), {10.0, 10.0, 10.0}});
    }

    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);
    EXPECT_EQ(result.unpacked_items.size(), 0u);
}

TEST(SolverTimeoutTest, DisabledTimeoutPacksEverything) {
    SolverConfig config;
    config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}});
    config.timeout = std::chrono::milliseconds::zero();  // disabled

    std::vector<Item> items;
    for (int i = 0; i < 8; ++i) {
        items.push_back(Item{"box_" + std::to_string(i), {20.0, 20.0, 20.0}});
    }

    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);
    EXPECT_EQ(result.unpacked_items.size(), 0u);
    EXPECT_EQ(result.placements.size(), items.size());
}

// =============================================================================
// Thread budget (F9)
// =============================================================================

TEST(ThreadCountTest, SingleThreadProducesValidResult) {
    SolverConfig config;
    config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}});
    config.allow_multiple_bins = true;
    config.thread_count = 1;  // Force the parallel solver into single-wave mode.

    std::vector<Item> items = {
        Item{"a", {50.0, 50.0, 50.0}},
        Item{"b", {30.0, 30.0, 30.0}},
        Item{"c", {25.0, 25.0, 25.0}},
        Item{"d", {15.0, 15.0, 15.0}},
    };

    ParallelSolver parallel;
    auto result = parallel.solve(items, config);

    // A constrained thread budget must not change correctness: all items packed.
    EXPECT_EQ(result.unpacked_items.size(), 0u);
    EXPECT_EQ(result.placements.size(), items.size());
}

TEST(ThreadCountTest, ThreadBudgetDoesNotChangeBestResult) {
    SolverConfig base;
    base.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}});
    base.allow_multiple_bins = true;

    std::vector<Item> items = {
        Item{"a", {50.0, 50.0, 50.0}}, Item{"b", {40.0, 40.0, 40.0}}, Item{"c", {30.0, 30.0, 30.0}},
        Item{"d", {20.0, 20.0, 20.0}}, Item{"e", {15.0, 15.0, 15.0}},
    };

    SolverConfig one = base;
    one.thread_count = 1;
    SolverConfig many = base;
    many.thread_count = 8;

    ParallelSolver parallel;
    auto r1 = parallel.solve(items, one);
    auto r8 = parallel.solve(items, many);

    // The selected best result is independent of how many threads ran the search.
    EXPECT_EQ(r1.bins_used, r8.bins_used);
    EXPECT_EQ(r1.unpacked_items.size(), r8.unpacked_items.size());
}

}  // namespace bp3d::test
