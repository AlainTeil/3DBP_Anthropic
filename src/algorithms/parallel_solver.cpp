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
#include "bp3d/config.hpp"
#include "bp3d/logger.hpp"
#include "bp3d/result.hpp"
#include "bp3d/solver.hpp"
#include "bp3d/types.hpp"

#include <algorithm>
#include <cstddef>
#include <exception>
#include <future>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace bp3d {

ParallelSolver::ParallelSolver() {
    // Default: run all offline algorithms
    algorithms_ = {Algorithm::FirstFitDecreasing, Algorithm::BestFitDecreasing,
                   Algorithm::Guillotine, Algorithm::ExtremePoint, Algorithm::Shelf};
}

ParallelSolver::ParallelSolver(std::vector<Algorithm> algorithms)
    : algorithms_(std::move(algorithms)) {}

bool is_better_result(const PackingResult& a, const PackingResult& b, PackingObjective objective) {
    // A more complete packing always wins, regardless of objective.
    if (a.unpacked_items.size() != b.unpacked_items.size()) {
        return a.unpacked_items.size() < b.unpacked_items.size();
    }

    switch (objective) {
        case PackingObjective::MinimizeBins:
            if (a.bins_used != b.bins_used) {
                return a.bins_used < b.bins_used;
            }
            return a.stats.utilization > b.stats.utilization;

        case PackingObjective::MinimizeCost:
            if (a.stats.total_cost != b.stats.total_cost) {
                return a.stats.total_cost < b.stats.total_cost;
            }
            if (a.bins_used != b.bins_used) {
                return a.bins_used < b.bins_used;
            }
            return a.stats.utilization > b.stats.utilization;

        case PackingObjective::MaximizeUtilization:
            if (a.stats.utilization != b.stats.utilization) {
                return a.stats.utilization > b.stats.utilization;
            }
            return a.bins_used < b.bins_used;
    }

    // Unreachable; fall back to bin count.
    return a.bins_used < b.bins_used;
}

bool is_better_result(const PackingResult& a, const PackingResult& b) {
    return is_better_result(a, b, PackingObjective::MinimizeBins);
}

PackingResult ParallelSolver::solve(std::span<const Item> items, const SolverConfig& config) {
    if (algorithms_.empty() || items.empty() || config.bin_types.empty()) {
        PackingResult empty_result;
        empty_result.algorithm = std::string(name());
        return empty_result;
    }

    // Convert items to vector for thread safety
    std::vector<Item> items_copy(items.begin(), items.end());

    // Run algorithms concurrently, but cap the number of simultaneously running
    // solvers at the configured thread budget. Algorithms are dispatched in
    // waves of at most effective_thread_count() futures.
    const auto max_concurrency =
        std::max<std::size_t>(1, static_cast<std::size_t>(config.effective_thread_count()));

    std::vector<PackingResult> results;
    results.reserve(algorithms_.size());

    for (std::size_t wave_start = 0; wave_start < algorithms_.size();
         wave_start += max_concurrency) {
        const std::size_t wave_end = std::min(wave_start + max_concurrency, algorithms_.size());

        std::vector<std::future<PackingResult>> futures;
        futures.reserve(wave_end - wave_start);
        for (std::size_t k = wave_start; k < wave_end; ++k) {
            const Algorithm algo = algorithms_[k];
            futures.push_back(std::async(std::launch::async, [algo, &items_copy, &config]() {
                auto solver = create_solver(algo);
                return solver->solve(items_copy, config);
            }));
        }

        // Collect this wave's results. A solver that throws is logged and
        // skipped rather than silently ignored, so failures are diagnosable.
        for (std::size_t k = 0; k < futures.size(); ++k) {
            const std::string algo_name{algorithm_name(algorithms_[wave_start + k])};
            try {
                results.push_back(futures[k].get());
            } catch (const std::exception& e) {
                Logger::warning("Algorithm '" + algo_name + "' failed: " + e.what());
            } catch (...) {
                Logger::warning("Algorithm '" + algo_name + "' failed with an unknown exception");
            }
        }
    }

    if (results.empty()) {
        PackingResult empty_result;
        empty_result.algorithm = std::string(name());
        return empty_result;
    }

    // Find best result
    PackingResult* best = results.data();
    for (std::size_t i = 1; i < results.size(); ++i) {
        if (is_better_result(results[i], *best, config.objective)) {
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
