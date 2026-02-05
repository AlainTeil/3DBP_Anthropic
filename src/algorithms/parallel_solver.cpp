/**
 * @file parallel_solver.cpp
 * @brief ParallelSolver implementation
 */

#include "bp3d/algorithms/parallel_solver.hpp"

#include "bp3d/algorithms/bfd.hpp"
#include "bp3d/algorithms/extreme_point.hpp"
#include "bp3d/algorithms/ffd.hpp"
#include "bp3d/algorithms/guillotine.hpp"
#include "bp3d/algorithms/shelf.hpp"

#include <future>
#include <mutex>
#include <thread>
#include <vector>

namespace bp3d {

ParallelSolver::ParallelSolver() {
    // Default: run all offline algorithms
    algorithms_ = {Algorithm::FirstFitDecreasing, Algorithm::BestFitDecreasing,
                   Algorithm::Guillotine, Algorithm::ExtremePoint, Algorithm::Shelf};
}

ParallelSolver::ParallelSolver(std::vector<Algorithm> algorithms)
    : algorithms_(std::move(algorithms)) {}

bool is_better_result(const PackingResult& a, const PackingResult& b) {
    // First: fewer bins is better
    if (a.bins_used != b.bins_used) {
        return a.bins_used < b.bins_used;
    }

    // Second: fewer unpacked items is better
    if (a.unpacked_items.size() != b.unpacked_items.size()) {
        return a.unpacked_items.size() < b.unpacked_items.size();
    }

    // Third: higher utilization is better
    return a.stats.utilization > b.stats.utilization;
}

PackingResult ParallelSolver::solve(std::span<const Item> items, const SolverConfig& config) {
    if (algorithms_.empty() || items.empty() || config.bin_types.empty()) {
        PackingResult empty_result;
        empty_result.algorithm = std::string(name());
        return empty_result;
    }

    // Convert items to vector for thread safety
    std::vector<Item> items_copy(items.begin(), items.end());

    // Run algorithms in parallel
    std::vector<std::future<PackingResult>> futures;
    futures.reserve(algorithms_.size());

    for (Algorithm algo : algorithms_) {
        futures.push_back(std::async(std::launch::async, [algo, &items_copy, &config]() {
            auto solver = create_solver(algo);
            return solver->solve(items_copy, config);
        }));
    }

    // Collect results
    std::vector<PackingResult> results;
    results.reserve(futures.size());

    for (auto& future : futures) {
        try {
            results.push_back(future.get());
        } catch (...) {
            // Ignore failed solvers
        }
    }

    if (results.empty()) {
        PackingResult empty_result;
        empty_result.algorithm = std::string(name());
        return empty_result;
    }

    // Find best result
    PackingResult* best = &results[0];
    for (std::size_t i = 1; i < results.size(); ++i) {
        if (is_better_result(results[i], *best)) {
            best = &results[i];
        }
    }

    // Mark result as coming from parallel solver
    PackingResult final_result = std::move(*best);
    final_result.algorithm = std::string(name()) + " (" + final_result.algorithm + ")";

    return final_result;
}

// ============================================================================
// Solver factory implementations
// ============================================================================

std::unique_ptr<ISolver> create_solver(Algorithm algorithm) {
    switch (algorithm) {
        case Algorithm::FirstFitDecreasing:
            return std::make_unique<FirstFitDecreasing>();
        case Algorithm::BestFitDecreasing:
            return std::make_unique<BestFitDecreasing>();
        case Algorithm::Guillotine:
            return std::make_unique<GuillotineSolver>();
        case Algorithm::ExtremePoint:
            return std::make_unique<ExtremePointSolver>();
        case Algorithm::Shelf:
            return std::make_unique<ShelfSolver>();
    }
    throw std::invalid_argument("Unknown algorithm");
}

std::unique_ptr<IOnlineSolver> create_online_solver(Algorithm algorithm) {
    switch (algorithm) {
        case Algorithm::ExtremePoint:
            return std::make_unique<ExtremePointSolver>();
        case Algorithm::Shelf:
            return std::make_unique<ShelfSolver>();
        case Algorithm::FirstFitDecreasing:
        case Algorithm::BestFitDecreasing:
        case Algorithm::Guillotine:
            throw std::invalid_argument(std::string(algorithm_name(algorithm)) +
                                        " does not support online packing");
    }
    throw std::invalid_argument("Unknown algorithm");
}

bool algorithm_supports_online(Algorithm algorithm) {
    switch (algorithm) {
        case Algorithm::ExtremePoint:
        case Algorithm::Shelf:
            return true;
        case Algorithm::FirstFitDecreasing:
        case Algorithm::BestFitDecreasing:
        case Algorithm::Guillotine:
            return false;
    }
    return false;
}

}  // namespace bp3d
