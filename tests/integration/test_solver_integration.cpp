/**
 * @file test_solver_integration.cpp
 * @brief Integration tests for solver pipelines with constraints
 */

#include <bp3d/algorithms/bfd.hpp>
#include <bp3d/algorithms/extreme_point.hpp>
#include <bp3d/algorithms/ffd.hpp>
#include <bp3d/algorithms/guillotine.hpp>
#include <bp3d/algorithms/shelf.hpp>
#include <bp3d/bp3d.hpp>
#include <bp3d/constraints/collision.hpp>
#include <bp3d/constraints/stacking.hpp>
#include <bp3d/constraints/validator.hpp>
#include <bp3d/constraints/weight.hpp>

#include <gtest/gtest.h>

namespace bp3d::test {

/**
 * @brief Integration tests for algorithm + constraint validation
 */
class SolverConstraintIntegrationTest : public ::testing::Test {
protected:
    SolverConfig config;

    void SetUp() override {
        config.bin_types.push_back(BinType{"standard", {100.0, 100.0, 100.0}});
        config.allow_multiple_bins = true;
    }
};

TEST_F(SolverConstraintIntegrationTest, FFD_NoCollisions) {
    std::vector<Item> items;
    for (int i = 0; i < 10; ++i) {
        items.push_back(Item{"item_" + std::to_string(i), {20.0, 20.0, 20.0}});
    }

    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);

    // Verify no overlaps
    CollisionValidator validator;
    for (size_t i = 0; i < result.placements.size(); ++i) {
        for (size_t j = i + 1; j < result.placements.size(); ++j) {
            if (result.placements[i].bin_index == result.placements[j].bin_index) {
                EXPECT_FALSE(placements_overlap(result.placements[i], result.placements[j]))
                    << "Items " << result.placements[i].item_id << " and "
                    << result.placements[j].item_id << " overlap";
            }
        }
    }
}

TEST_F(SolverConstraintIntegrationTest, BFD_NoCollisions) {
    std::vector<Item> items;
    for (int i = 0; i < 10; ++i) {
        items.push_back(Item{"item_" + std::to_string(i), {20.0, 20.0, 20.0}});
    }

    BestFitDecreasing solver;
    auto result = solver.solve(items, config);

    for (size_t i = 0; i < result.placements.size(); ++i) {
        for (size_t j = i + 1; j < result.placements.size(); ++j) {
            if (result.placements[i].bin_index == result.placements[j].bin_index) {
                EXPECT_FALSE(placements_overlap(result.placements[i], result.placements[j]));
            }
        }
    }
}

TEST_F(SolverConstraintIntegrationTest, Guillotine_NoCollisions) {
    std::vector<Item> items;
    for (int i = 0; i < 10; ++i) {
        items.push_back(Item{"item_" + std::to_string(i), {20.0, 20.0, 20.0}});
    }

    GuillotineSolver solver;
    auto result = solver.solve(items, config);

    for (size_t i = 0; i < result.placements.size(); ++i) {
        for (size_t j = i + 1; j < result.placements.size(); ++j) {
            if (result.placements[i].bin_index == result.placements[j].bin_index) {
                EXPECT_FALSE(placements_overlap(result.placements[i], result.placements[j]));
            }
        }
    }
}

TEST_F(SolverConstraintIntegrationTest, AllPlacementsWithinBounds) {
    std::vector<Item> items;
    for (int i = 0; i < 10; ++i) {
        items.push_back(Item{"item_" + std::to_string(i), {20.0, 20.0, 20.0}});
    }

    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);

    for (const auto& p : result.placements) {
        auto bin_dims = config.bin_types[0].dimensions;

        EXPECT_GE(p.position.x, 0.0);
        EXPECT_GE(p.position.y, 0.0);
        EXPECT_GE(p.position.z, 0.0);

        EXPECT_LE(p.position.x + p.rotated_dimensions.width, bin_dims.width);
        EXPECT_LE(p.position.y + p.rotated_dimensions.height, bin_dims.height);
        EXPECT_LE(p.position.z + p.rotated_dimensions.depth, bin_dims.depth);
    }
}

TEST_F(SolverConstraintIntegrationTest, ThisSideUpRespected) {
    std::vector<Item> items;
    Item fragile{"fragile", {20.0, 30.0, 40.0}};
    fragile.constraints.this_side_up = true;
    items.push_back(fragile);

    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);

    ASSERT_EQ(result.placements.size(), 1);

    // When this_side_up is set, height should remain oriented correctly
    // The rotated dimensions should match one of the valid orientations
    auto& p = result.placements[0];
    auto& dims = p.rotated_dimensions;
    auto& orig = fragile.dimensions;

    // Height should be preserved (30.0)
    EXPECT_DOUBLE_EQ(dims.height, orig.height);
}

TEST_F(SolverConstraintIntegrationTest, WeightConstraint_BinMaxWeight) {
    SolverConfig weight_config;
    BinType bin{"limited", {100.0, 100.0, 100.0}};
    bin.max_weight = 50.0;
    weight_config.bin_types.push_back(bin);
    weight_config.allow_multiple_bins = true;

    std::vector<Item> items;
    for (int i = 0; i < 5; ++i) {
        Item item{"heavy_" + std::to_string(i), {20.0, 20.0, 20.0}};
        item.weight = 20.0;
        items.push_back(item);
    }

    FirstFitDecreasing solver;
    auto result = solver.solve(items, weight_config);

    // Should use multiple bins due to weight constraint
    // Total weight = 100, max per bin = 50, so need at least 2 bins
    EXPECT_GE(result.bins_used, 2);
}

TEST_F(SolverConstraintIntegrationTest, MixedConstraints) {
    std::vector<Item> items;

    // Fragile item that must stay upright
    Item fragile{"fragile", {10.0, 30.0, 10.0}};
    fragile.constraints.this_side_up = true;
    items.push_back(fragile);

    // Heavy item
    Item heavy{"heavy", {15.0, 15.0, 15.0}};
    heavy.weight = 50.0;
    items.push_back(heavy);

    // Normal items
    for (int i = 0; i < 5; ++i) {
        items.push_back(Item{"normal_" + std::to_string(i), {10.0, 10.0, 10.0}});
    }

    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);

    // All items should be packed
    EXPECT_EQ(result.placements.size(), items.size());
}

/**
 * @brief Integration test for quantity expansion + sorting + packing
 */
TEST_F(SolverConstraintIntegrationTest, QuantityExpansionIntegration) {
    std::vector<Item> items;
    Item multi{"multi", {10.0, 10.0, 10.0}};
    multi.quantity = 5;
    items.push_back(multi);

    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);

    // Should have 5 placements
    EXPECT_EQ(result.placements.size(), 5);

    // All should be in same bin (they fit)
    EXPECT_EQ(result.bins_used, 1);
}

/**
 * @brief Test different algorithms produce valid results
 */
class AlgorithmValidationTest : public ::testing::Test {
protected:
    SolverConfig config;
    std::vector<Item> items;

    void SetUp() override {
        config.bin_types.push_back(BinType{"standard", {100.0, 100.0, 100.0}});
        config.allow_multiple_bins = true;

        // Create varied items
        items = {
            Item{"large", {50.0, 50.0, 50.0}},   Item{"medium1", {30.0, 30.0, 30.0}},
            Item{"medium2", {25.0, 25.0, 25.0}}, Item{"small1", {15.0, 15.0, 15.0}},
            Item{"small2", {10.0, 10.0, 10.0}},
        };
    }

    void validate_result(const PackingResult& result, const std::string& algorithm) {
        // Check no overlaps
        for (size_t i = 0; i < result.placements.size(); ++i) {
            for (size_t j = i + 1; j < result.placements.size(); ++j) {
                if (result.placements[i].bin_index == result.placements[j].bin_index) {
                    EXPECT_FALSE(placements_overlap(result.placements[i], result.placements[j]))
                        << algorithm << ": Overlap found";
                }
            }
        }

        // Check bounds
        for (const auto& p : result.placements) {
            EXPECT_GE(p.position.x, 0.0) << algorithm << ": Negative X";
            EXPECT_GE(p.position.y, 0.0) << algorithm << ": Negative Y";
            EXPECT_GE(p.position.z, 0.0) << algorithm << ": Negative Z";
        }
    }
};

TEST_F(AlgorithmValidationTest, FFDProducesValidResult) {
    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);
    validate_result(result, "FFD");
}

TEST_F(AlgorithmValidationTest, BFDProducesValidResult) {
    BestFitDecreasing solver;
    auto result = solver.solve(items, config);
    validate_result(result, "BFD");
}

TEST_F(AlgorithmValidationTest, GuillotineProducesValidResult) {
    GuillotineSolver solver;
    auto result = solver.solve(items, config);
    validate_result(result, "Guillotine");
    EXPECT_GT(result.placements.size(), 0);
}

TEST_F(AlgorithmValidationTest, ExtremePointProducesValidResult) {
    ExtremePointSolver solver;
    auto result = solver.solve(items, config);
    validate_result(result, "ExtremePoint");
}

TEST_F(AlgorithmValidationTest, ShelfProducesValidResult) {
    ShelfSolver solver;
    auto result = solver.solve(items, config);
    validate_result(result, "Shelf");
}

/**
 * @brief Test composite validator with algorithms
 */
class CompositeValidatorIntegrationTest : public ::testing::Test {
protected:
    CompositeValidator validator;
    BinType bin{"standard", {100.0, 100.0, 100.0}};

    void SetUp() override {
        validator.add(std::make_unique<CollisionValidator>());
        validator.add(std::make_unique<StackingValidator>());
    }
};

TEST_F(CompositeValidatorIntegrationTest, AllValidatorsPass) {
    Item item{"test", {10.0, 10.0, 10.0}};
    std::vector<Placement> existing;
    Placement candidate{"test", "bin", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_TRUE(validator.can_place(item, candidate, existing, bin));
}

TEST_F(CompositeValidatorIntegrationTest, CollisionValidatorBlocks) {
    Item item{"test", {10.0, 10.0, 10.0}};
    std::vector<Placement> existing;
    existing.push_back(
        Placement{"other", "bin", 0, {0.0, 0.0, 0.0}, Rotation::WHD, {10.0, 10.0, 10.0}});

    // Try to place at same position
    Placement candidate{"test", "bin", 0, {5.0, 5.0, 5.0}, Rotation::WHD, {10.0, 10.0, 10.0}};

    EXPECT_FALSE(validator.can_place(item, candidate, existing, bin));
}

}  // namespace bp3d::test
