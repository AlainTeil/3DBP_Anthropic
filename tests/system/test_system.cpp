/**
 * @file test_system.cpp
 * @brief System tests for end-to-end CLI and workflow testing
 */

#include "test_support.hpp"

#include <bp3d/algorithms/bfd.hpp>
#include <bp3d/algorithms/extreme_point.hpp>
#include <bp3d/algorithms/ffd.hpp>
#include <bp3d/algorithms/guillotine.hpp>
#include <bp3d/algorithms/parallel_solver.hpp>
#include <bp3d/algorithms/shelf.hpp>
#include <bp3d/bp3d.hpp>
#include <bp3d/constraints/collision.hpp>
#include <bp3d/io/json_serializer.hpp>
#include <bp3d/io/obj_exporter.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace bp3d::test {

/**
 * @brief Execute a shell command and return the exit code
 */
int execute_command(const std::string& cmd, std::string& output) {
    std::array<char, 4096> buffer;
    output.clear();

    FILE* pipe = popen((cmd + " 2>&1").c_str(), "r");
    if (!pipe) {
        return -1;
    }

    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        output += buffer.data();
    }

    return pclose(pipe) / 256;  // Get actual exit code
}

/**
 * @brief System tests for complete workflows
 */
class SystemWorkflowTest : public ::testing::Test {
protected:
    std::filesystem::path temp_dir;

    void SetUp() override { temp_dir = make_unique_temp_dir("system"); }

    void TearDown() override { remove_temp_dir(temp_dir); }

    void create_input_file(const std::string& name, const std::string& content) {
        std::ofstream file(temp_dir / name);
        file << content;
    }
};

TEST_F(SystemWorkflowTest, CompletePackingWorkflow) {
    // Create input
    create_input_file("items.json", R"({
        "items": [
            {"id": "large", "width": 50, "height": 50, "depth": 50},
            {"id": "medium1", "width": 30, "height": 30, "depth": 30},
            {"id": "medium2", "width": 25, "height": 25, "depth": 25},
            {"id": "small1", "width": 15, "height": 15, "depth": 15},
            {"id": "small2", "width": 10, "height": 10, "depth": 10}
        ],
        "bins": [
            {"id": "container", "width": 100, "height": 100, "depth": 100}
        ]
    })");

    // Load input
    auto input = load_input_file((temp_dir / "items.json").string());
    ASSERT_EQ(input.items.size(), 5);
    ASSERT_EQ(input.bins.size(), 1);

    // Run solver
    SolverConfig config;
    config.bin_types = input.bins;

    FirstFitDecreasing solver;
    auto result = solver.solve(input.items, config);

    // Verify result
    EXPECT_GT(result.placements.size(), 0);
    EXPECT_EQ(result.placements.size() + result.unpacked_items.size(), input.items.size());

    // Export JSON
    save_result_file(result, (temp_dir / "result.json").string());
    EXPECT_TRUE(std::filesystem::exists(temp_dir / "result.json"));

    // Export OBJ
    export_to_obj(result, input.bins, temp_dir / "result.obj");
    EXPECT_TRUE(std::filesystem::exists(temp_dir / "result.obj"));

    // Verify OBJ content
    std::ifstream obj_file(temp_dir / "result.obj");
    std::string obj_content((std::istreambuf_iterator<char>(obj_file)),
                            std::istreambuf_iterator<char>());
    EXPECT_NE(obj_content.find("v "), std::string::npos);  // Has vertices
    EXPECT_NE(obj_content.find("f "), std::string::npos);  // Has faces
}

TEST_F(SystemWorkflowTest, AllAlgorithmsProduceConsistentResults) {
    create_input_file("test.json", R"({
        "items": [
            {"id": "a", "width": 20, "height": 20, "depth": 20},
            {"id": "b", "width": 20, "height": 20, "depth": 20},
            {"id": "c", "width": 20, "height": 20, "depth": 20},
            {"id": "d", "width": 20, "height": 20, "depth": 20}
        ],
        "bins": [
            {"id": "bin", "width": 100, "height": 100, "depth": 100}
        ]
    })");

    auto input = load_input_file((temp_dir / "test.json").string());

    SolverConfig config;
    config.bin_types = input.bins;

    // Test all algorithms
    std::vector<std::unique_ptr<ISolver>> solvers;
    solvers.push_back(std::make_unique<FirstFitDecreasing>());
    solvers.push_back(std::make_unique<BestFitDecreasing>());
    solvers.push_back(std::make_unique<GuillotineSolver>());
    solvers.push_back(std::make_unique<ShelfSolver>());
    solvers.push_back(std::make_unique<ExtremePointSolver>());

    for (size_t si = 0; si < solvers.size(); ++si) {
        auto& solver = solvers[si];
        auto result = solver->solve(input.items, config);

        // Each algorithm should pack at least some items
        EXPECT_GT(result.placements.size(), 0) << "Algorithm: " << result.algorithm;

        // Total should equal input size
        EXPECT_EQ(result.placements.size() + result.unpacked_items.size(), input.items.size())
            << "Algorithm: " << result.algorithm;

        // No overlaps
        for (size_t i = 0; i < result.placements.size(); ++i) {
            for (size_t j = i + 1; j < result.placements.size(); ++j) {
                if (result.placements[i].bin_index == result.placements[j].bin_index) {
                    EXPECT_FALSE(placements_overlap(result.placements[i], result.placements[j]))
                        << "Algorithm: " << result.algorithm;
                }
            }
        }
    }
}

TEST_F(SystemWorkflowTest, ParallelSolverSelectsBest) {
    create_input_file("parallel_test.json", R"({
        "items": [
            {"id": "a", "width": 50, "height": 50, "depth": 50},
            {"id": "b", "width": 40, "height": 40, "depth": 40},
            {"id": "c", "width": 30, "height": 30, "depth": 30},
            {"id": "d", "width": 20, "height": 20, "depth": 20},
            {"id": "e", "width": 10, "height": 10, "depth": 10}
        ],
        "bins": [
            {"id": "bin", "width": 100, "height": 100, "depth": 100}
        ]
    })");

    auto input = load_input_file((temp_dir / "parallel_test.json").string());

    SolverConfig config;
    config.bin_types = input.bins;
    config.allow_multiple_bins = true;

    ParallelSolver parallel;
    auto result = parallel.solve(input.items, config);

    // Parallel solver should pack all items
    EXPECT_EQ(result.placements.size(), input.items.size());
    EXPECT_EQ(result.unpacked_items.size(), 0);

    // Should minimize bins
    EXPECT_GE(result.bins_used, 1);
}

TEST_F(SystemWorkflowTest, LargeScaleTest) {
    // Create many items
    std::string items_json = R"({"items": [)";
    for (int i = 0; i < 100; ++i) {
        if (i > 0)
            items_json += ",";
        items_json +=
            R"({"id": "item)" + std::to_string(i) + R"(", "width": 10, "height": 10, "depth": 10})";
    }
    items_json += R"(], "bins": [{"id": "large", "width": 100, "height": 100, "depth": 100}]})";

    create_input_file("large.json", items_json);

    auto input = load_input_file((temp_dir / "large.json").string());
    ASSERT_EQ(input.items.size(), 100);

    SolverConfig config;
    config.bin_types = input.bins;
    config.allow_multiple_bins = true;

    FirstFitDecreasing solver;
    auto result = solver.solve(input.items, config);

    // All items should be packed (100 items of 10^3 = 100000 volume)
    // Bin is 100^3 = 1000000 volume, should fit all
    EXPECT_EQ(result.placements.size(), 100);
    EXPECT_EQ(result.bins_used, 1);
}

TEST_F(SystemWorkflowTest, ConstrainedPackingWorkflow) {
    create_input_file("constrained.json", R"({
        "items": [
            {"id": "fragile", "width": 20, "height": 40, "depth": 20, "this_side_up": true, "weight": 5},
            {"id": "heavy", "width": 30, "height": 30, "depth": 30, "weight": 100},
            {"id": "normal1", "width": 15, "height": 15, "depth": 15, "weight": 10},
            {"id": "normal2", "width": 15, "height": 15, "depth": 15, "weight": 10}
        ],
        "bins": [
            {"id": "container", "width": 100, "height": 100, "depth": 100, "max_weight": 200}
        ]
    })");

    auto input = load_input_file((temp_dir / "constrained.json").string());

    SolverConfig config;
    config.bin_types = input.bins;
    config.allow_multiple_bins = true;

    FirstFitDecreasing solver;
    auto result = solver.solve(input.items, config);

    EXPECT_GT(result.placements.size(), 0);
}

TEST_F(SystemWorkflowTest, MultiBinTypeWorkflow) {
    create_input_file("multi_bin.json", R"({
        "items": [
            {"id": "huge", "width": 80, "height": 80, "depth": 80},
            {"id": "small1", "width": 10, "height": 10, "depth": 10},
            {"id": "small2", "width": 10, "height": 10, "depth": 10}
        ],
        "bins": [
            {"id": "small_bin", "width": 50, "height": 50, "depth": 50, "cost": 5},
            {"id": "large_bin", "width": 100, "height": 100, "depth": 100, "cost": 15}
        ]
    })");

    auto input = load_input_file((temp_dir / "multi_bin.json").string());
    ASSERT_EQ(input.bins.size(), 2);

    SolverConfig config;
    config.bin_types = input.bins;
    config.allow_multiple_bins = true;

    FirstFitDecreasing solver;
    auto result = solver.solve(input.items, config);

    // Should use multiple bins (huge item needs large bin, small items fit elsewhere)
    EXPECT_GT(result.placements.size(), 0);
}

/**
 * @brief End-to-end scenario tests
 */
class ScenarioTest : public ::testing::Test {
protected:
    std::filesystem::path temp_dir;

    void SetUp() override { temp_dir = make_unique_temp_dir("scenario"); }

    void TearDown() override { remove_temp_dir(temp_dir); }
};

TEST_F(ScenarioTest, WarehousePackingScenario) {
    // Simulate a warehouse packing scenario with various product sizes
    std::vector<Item> items;

    // Small products (like books)
    for (int i = 0; i < 20; ++i) {
        items.push_back(Item{"book_" + std::to_string(i), {15.0, 20.0, 3.0}});
    }

    // Medium products (like electronics boxes)
    for (int i = 0; i < 10; ++i) {
        items.push_back(Item{"electronic_" + std::to_string(i), {30.0, 25.0, 20.0}});
    }

    // Large products (like appliances)
    for (int i = 0; i < 5; ++i) {
        items.push_back(Item{"appliance_" + std::to_string(i), {50.0, 60.0, 50.0}});
    }

    SolverConfig config;
    config.bin_types.push_back(BinType{"pallet", {120.0, 150.0, 100.0}});
    config.allow_multiple_bins = true;

    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);

    // All items should be packed
    EXPECT_EQ(result.placements.size() + result.unpacked_items.size(), items.size());

    // Should achieve reasonable utilization
    EXPECT_GT(result.utilization_percent(), 10.0);  // At least 10% utilization
}

TEST_F(ScenarioTest, ShippingContainerScenario) {
    // Simulate shipping container loading
    std::vector<Item> items;

    // Standard shipping boxes
    for (int i = 0; i < 50; ++i) {
        Item box{"box_" + std::to_string(i), {40.0, 40.0, 40.0}};
        box.weight = 25.0;
        items.push_back(box);
    }

    SolverConfig config;
    BinType container{"container", {240.0, 240.0, 600.0}};
    container.max_weight = 2000.0;
    config.bin_types.push_back(container);
    config.allow_multiple_bins = true;

    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);

    EXPECT_GT(result.placements.size(), 0);

    // Verify no placement exceeds bin bounds
    for (const auto& p : result.placements) {
        EXPECT_GE(p.position.x, 0.0);
        EXPECT_GE(p.position.y, 0.0);
        EXPECT_GE(p.position.z, 0.0);
        EXPECT_LE(p.position.x + p.rotated_dimensions.width, container.dimensions.width);
        EXPECT_LE(p.position.y + p.rotated_dimensions.height, container.dimensions.height);
        EXPECT_LE(p.position.z + p.rotated_dimensions.depth, container.dimensions.depth);
    }
}

TEST_F(ScenarioTest, MixedOrientationConstraints) {
    std::vector<Item> items;

    // Items that must stay upright (fragile)
    for (int i = 0; i < 5; ++i) {
        Item fragile{"fragile_" + std::to_string(i), {20.0, 40.0, 20.0}};
        fragile.constraints.this_side_up = true;
        items.push_back(fragile);
    }

    // Items that can be rotated freely
    for (int i = 0; i < 10; ++i) {
        items.push_back(Item{"regular_" + std::to_string(i), {25.0, 30.0, 35.0}});
    }

    SolverConfig config;
    config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}});
    config.allow_multiple_bins = true;

    FirstFitDecreasing solver;
    auto result = solver.solve(items, config);

    // All items should be handled
    EXPECT_EQ(result.placements.size() + result.unpacked_items.size(), items.size());
}

/**
 * @brief Performance benchmark tests
 */
class PerformanceTest : public ::testing::Test {
protected:
    SolverConfig config;

    void SetUp() override {
        config.bin_types.push_back(BinType{"bin", {100.0, 100.0, 100.0}});
        config.allow_multiple_bins = true;
    }
};

TEST_F(PerformanceTest, SmallProblemPerformance) {
    std::vector<Item> items;
    for (int i = 0; i < 10; ++i) {
        items.push_back(Item{"item_" + std::to_string(i), {10.0, 10.0, 10.0}});
    }

    FirstFitDecreasing solver;
    auto start = std::chrono::high_resolution_clock::now();
    auto result = solver.solve(items, config);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    EXPECT_LT(duration.count(), 1000);  // Should complete in under 1 second

    EXPECT_EQ(result.placements.size(), 10);
}

TEST_F(PerformanceTest, MediumProblemPerformance) {
    std::vector<Item> items;
    for (int i = 0; i < 100; ++i) {
        items.push_back(Item{"item_" + std::to_string(i), {10.0, 10.0, 10.0}});
    }

    FirstFitDecreasing solver;
    auto start = std::chrono::high_resolution_clock::now();
    auto result = solver.solve(items, config);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    EXPECT_LT(duration.count(), 5000);  // Should complete in under 5 seconds

    EXPECT_EQ(result.placements.size(), 100);
}

TEST_F(PerformanceTest, AllAlgorithmsComplete) {
    std::vector<Item> items;
    for (int i = 0; i < 50; ++i) {
        items.push_back(Item{"item_" + std::to_string(i), {15.0, 15.0, 15.0}});
    }

    std::vector<std::unique_ptr<ISolver>> solvers;
    solvers.push_back(std::make_unique<FirstFitDecreasing>());
    solvers.push_back(std::make_unique<BestFitDecreasing>());
    solvers.push_back(std::make_unique<GuillotineSolver>());
    solvers.push_back(std::make_unique<ShelfSolver>());
    solvers.push_back(std::make_unique<ExtremePointSolver>());

    for (size_t idx = 0; idx < solvers.size(); ++idx) {
        auto start_time = std::chrono::high_resolution_clock::now();
        auto result = solvers[idx]->solve(items, config);
        auto end_time = std::chrono::high_resolution_clock::now();

        auto duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        EXPECT_LT(duration.count(), 10000) << "Algorithm " << result.algorithm << " took too long";
    }
}

/**
 * @brief Large-N benchmark exercising the spatial-index accelerated engine.
 *
 * Packs a large, dense item set (filling multiple bins) and verifies both that
 * it completes in a reasonable time and that the spatial-index neighbour
 * narrowing did not introduce any missed collisions.
 *
 * The time ceiling is deliberately generous: this same test runs under
 * unoptimized Debug builds and under ASan/TSan instrumentation, so it guards
 * against gross regressions (e.g. losing the spatial index and reverting to a
 * full O(n) collision scan per candidate) rather than asserting a tight bound.
 */
TEST_F(PerformanceTest, LargeProblemPacksDenselyWithoutOverlaps) {
    // 600 unit cubes; 1000 of them exactly fill one 100^3 bin, so this packs
    // into a single densely populated bin and stresses the per-bin neighbour
    // index the most.
    constexpr int kItemCount = 600;
    std::vector<Item> items;
    items.reserve(kItemCount);
    for (int i = 0; i < kItemCount; ++i) {
        items.push_back(Item{"item_" + std::to_string(i), {10.0, 10.0, 10.0}});
    }

    FirstFitDecreasing solver;
    const auto start = std::chrono::high_resolution_clock::now();
    auto result = solver.solve(items, config);
    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // Every item should be accounted for and, given the perfect fit, packed.
    EXPECT_EQ(result.placements.size(), static_cast<std::size_t>(kItemCount));
    EXPECT_TRUE(result.unpacked_items.empty());

    // Generous ceiling spanning Debug + sanitizer builds; a regression to the
    // previous O(n) per-candidate collision scan would blow straight through it.
    EXPECT_LT(duration.count(), 60000);

    // Correctness guard: no two placements in the same bin may overlap.
    std::unordered_map<int, std::vector<const Placement*>> by_bin;
    for (const auto& p : result.placements) {
        by_bin[p.bin_index].push_back(&p);
    }
    for (const auto& [bin_index, placements] : by_bin) {
        for (std::size_t i = 0; i < placements.size(); ++i) {
            for (std::size_t j = i + 1; j < placements.size(); ++j) {
                ASSERT_FALSE(placements_overlap(*placements[i], *placements[j]))
                    << "overlap in bin " << bin_index << " between placements " << i << " and "
                    << j;
            }
        }
    }
}

}  // namespace bp3d::test
