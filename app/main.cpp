/**
 * @file main.cpp
 * @brief CLI application for bp3d bin packing
 */

#include "bp3d/config.hpp"
#include "bp3d/io/json_serializer.hpp"
#include "bp3d/io/obj_exporter.hpp"
#include "bp3d/result.hpp"
#include "bp3d/rotation.hpp"
#include "bp3d/solver.hpp"
#include "bp3d/types.hpp"

#include <bp3d/algorithms/parallel_solver.hpp>

#include <chrono>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]\n\n"
              << "3D Bin Packing Tool\n\n"
              << "Options:\n"
              << "  -i, --input FILE       Input JSON file with items (required)\n"
              << "  -o, --output FILE      Output JSON file for results\n"
              << "  -a, --algorithm ALG    Algorithm to use:\n"
              << "                           ffd       - First Fit Decreasing\n"
              << "                           bfd       - Best Fit Decreasing\n"
              << "                           guillotine - Guillotine splitting\n"
              << "                           extreme   - Extreme Point\n"
              << "                           shelf     - Shelf/Layer based\n"
              << "                           parallel  - Run all, pick best (default)\n"
              << "  -c, --compare          Compare all algorithms\n"
              << "  --export-obj FILE      Export to Wavefront OBJ\n"
              << "  -t, --threads N        Number of threads (default: auto)\n"
              << "  -v, --verbose          Verbose output\n"
              << "  -h, --help             Show this help message\n\n"
              << "Example:\n"
              << "  " << program_name << " -i items.json -o result.json -a ffd\n";
}

bp3d::Algorithm parse_algorithm(const std::string& str) {
    if (str == "ffd" || str == "firstfit") {
        return bp3d::Algorithm::FirstFitDecreasing;
    }
    if (str == "bfd" || str == "bestfit") {
        return bp3d::Algorithm::BestFitDecreasing;
    }
    if (str == "guillotine") {
        return bp3d::Algorithm::Guillotine;
    }
    if (str == "extreme" || str == "extremepoint" || str == "ep") {
        return bp3d::Algorithm::ExtremePoint;
    }
    if (str == "shelf") {
        return bp3d::Algorithm::Shelf;
    }
    throw std::invalid_argument("Unknown algorithm: " + str);
}

void print_result(const bp3d::PackingResult& result, bool verbose) {
    std::cout << "\n=== Packing Result ===\n";
    std::cout << "Algorithm: " << result.algorithm << "\n";
    std::cout << "Bins used: " << result.bins_used << "\n";
    std::cout << "Items packed: " << result.placements.size() << "\n";
    std::cout << "Unpacked items: " << result.unpacked_items.size() << "\n";
    std::cout << "Utilization: " << std::fixed << std::setprecision(2)
              << result.utilization_percent() << "%\n";
    std::cout << "Execution time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(result.stats.execution_time)
                     .count()
              << " ms\n";

    if (!result.unpacked_items.empty()) {
        std::cout << "\nUnpacked items:";
        for (const auto& id : result.unpacked_items) {
            std::cout << " " << id;
        }
        std::cout << "\n";
    }

    if (verbose && !result.placements.empty()) {
        std::cout << "\nPlacements:\n";
        for (const auto& p : result.placements) {
            std::cout << "  " << p.item_id << " -> bin " << p.bin_id << "[" << p.bin_index
                      << "] at (" << p.position.x << ", " << p.position.y << ", " << p.position.z
                      << ") rot=" << bp3d::rotation_name(p.rotation) << "\n";
        }
    }
}

void compare_algorithms(const std::vector<bp3d::Item>& items, const bp3d::SolverConfig& config) {
    std::vector<bp3d::Algorithm> const algorithms = {
        bp3d::Algorithm::FirstFitDecreasing, bp3d::Algorithm::BestFitDecreasing,
        bp3d::Algorithm::Guillotine, bp3d::Algorithm::ExtremePoint, bp3d::Algorithm::Shelf};

    std::cout << "\n=== Algorithm Comparison ===\n";
    std::cout << std::setw(20) << "Algorithm" << std::setw(10) << "Bins" << std::setw(15)
              << "Utilization" << std::setw(15) << "Time (ms)" << std::setw(10) << "Unpacked"
              << "\n";
    std::cout << std::string(70, '-') << "\n";

    for (bp3d::Algorithm const algo : algorithms) {
        auto solver = bp3d::create_solver(algo);
        auto result = solver->solve(items, config);

        std::cout << std::setw(20) << std::string(bp3d::algorithm_name(algo)) << std::setw(10)
                  << result.bins_used << std::setw(14) << std::fixed << std::setprecision(1)
                  << result.utilization_percent() << "%" << std::setw(15)
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         result.stats.execution_time)
                         .count()
                  << std::setw(10) << result.unpacked_items.size() << "\n";
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string input_file;
    std::string output_file;
    std::string obj_file;
    std::string algorithm_str = "parallel";
    unsigned int threads = 0;
    bool verbose = false;
    bool compare = false;

    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        std::string const arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
        if (arg == "-v" || arg == "--verbose") {
            verbose = true;
            continue;
        }
        if (arg == "-c" || arg == "--compare") {
            compare = true;
            continue;
        }
        if ((arg == "-i" || arg == "--input") && i + 1 < argc) {
            input_file = argv[++i];
            continue;
        }
        if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
            output_file = argv[++i];
            continue;
        }
        if ((arg == "-a" || arg == "--algorithm") && i + 1 < argc) {
            algorithm_str = argv[++i];
            continue;
        }
        if (arg == "--export-obj" && i + 1 < argc) {
            obj_file = argv[++i];
            continue;
        }
        if ((arg == "-t" || arg == "--threads") && i + 1 < argc) {
            threads = static_cast<unsigned int>(std::stoi(argv[++i]));
            continue;
        }

        std::cerr << "Unknown option: " << arg << "\n";
        print_usage(argv[0]);
        return 1;
    }

    if (input_file.empty()) {
        std::cerr << "Error: Input file is required\n";
        print_usage(argv[0]);
        return 1;
    }

    try {
        // Load input
        std::cout << "Loading input from " << input_file << "...\n";
        auto input = bp3d::load_input_file(input_file);

        if (input.items.empty()) {
            std::cerr << "Error: No items found in input file\n";
            return 1;
        }

        std::cout << "Loaded " << input.items.size() << " item types\n";

        // Set up default bin if none specified
        if (input.bins.empty()) {
            std::cout << "No bins specified, using default 100x100x100\n";
            input.bins.push_back(bp3d::BinType{"default", {100.0, 100.0, 100.0}, 1000.0, 1.0});
        }

        std::cout << "Using " << input.bins.size() << " bin type(s)\n";

        // Configure solver
        bp3d::SolverConfig config;
        config.bin_types = input.bins;
        config.thread_count = threads;
        config.allow_multiple_bins = true;

        // Comparison mode
        if (compare) {
            compare_algorithms(input.items, config);
            return 0;
        }

        // Run solver
        bp3d::PackingResult result;
        if (algorithm_str == "parallel" || algorithm_str == "all") {
            bp3d::ParallelSolver solver;
            result = solver.solve(input.items, config);
        } else {
            auto algo = parse_algorithm(algorithm_str);
            auto solver = bp3d::create_solver(algo);
            result = solver->solve(input.items, config);
        }

        // Print result
        print_result(result, verbose);

        // Save JSON output
        if (!output_file.empty()) {
            bp3d::save_result_file(result, output_file);
            std::cout << "\nResult saved to " << output_file << "\n";
        }

        // Export OBJ
        if (!obj_file.empty()) {
            bp3d::export_to_obj(result, input.bins, obj_file);
            std::cout << "OBJ exported to " << obj_file << "\n";
        }

        return result.is_complete() ? 0 : 1;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
