/**
 * @file config.hpp
 * @brief Solver configuration types
 */

#ifndef BP3D_CONFIG_HPP
#define BP3D_CONFIG_HPP

#include "bp3d/types.hpp"

#include <chrono>
#include <thread>
#include <vector>

namespace bp3d {

/**
 * @brief Available packing algorithms
 */
enum class Algorithm {
    FirstFitDecreasing,  ///< FFD - Sort by volume, first fit
    BestFitDecreasing,   ///< BFD - Sort by volume, best fit
    Guillotine,          ///< Guillotine splitting algorithm
    ExtremePoint,        ///< Extreme point based placement
    Shelf                ///< Shelf/layer based packing
};

/**
 * @brief Get string name of an algorithm
 */
[[nodiscard]] constexpr std::string_view algorithm_name(Algorithm algo) noexcept {
    switch (algo) {
        case Algorithm::FirstFitDecreasing:
            return "FirstFitDecreasing";
        case Algorithm::BestFitDecreasing:
            return "BestFitDecreasing";
        case Algorithm::Guillotine:
            return "Guillotine";
        case Algorithm::ExtremePoint:
            return "ExtremePoint";
        case Algorithm::Shelf:
            return "Shelf";
    }
    return "Unknown";
}

/**
 * @brief Optimization objective used to choose the best packing.
 *
 * Selects how the parallel solver ranks the results produced by its candidate
 * algorithms. In every case a more complete packing (fewer unpacked items) is
 * preferred first; the objective then decides among equally complete results.
 */
enum class PackingObjective {
    MinimizeBins,        ///< Prefer the fewest bins, then highest utilization
    MinimizeCost,        ///< Prefer the lowest total bin cost, then fewest bins
    MaximizeUtilization  ///< Prefer the highest volume utilization, then fewest bins
};

/**
 * @brief Configuration for the packing solver
 */
struct SolverConfig {
    /// Available bin types to use for packing
    std::vector<BinType> bin_types;

    /// Whether more than one bin may be opened. When false, packing is limited
    /// to a single bin and any items that do not fit are reported as unpacked.
    bool allow_multiple_bins = true;

    /// Optimization objective the parallel solver uses to pick the best result.
    PackingObjective objective = PackingObjective::MinimizeBins;

    /// Maximum wall-clock time to spend packing. Solvers check this between
    /// items and stop placing once it is exceeded, reporting any remaining
    /// items as unpacked. A non-positive value disables the limit.
    std::chrono::milliseconds timeout{std::chrono::minutes(5)};

    /// Number of threads to use (0 = auto-detect). Bounds how many algorithms
    /// the parallel solver runs concurrently.
    unsigned int thread_count = 0;

    /// Get effective thread count
    [[nodiscard]] unsigned int effective_thread_count() const noexcept {
        if (thread_count == 0) {
            unsigned int const hw = std::thread::hardware_concurrency();
            return hw > 0 ? hw : 1;
        }
        return thread_count;
    }
};

}  // namespace bp3d

#endif  // BP3D_CONFIG_HPP
