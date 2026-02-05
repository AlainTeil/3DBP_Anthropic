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
 * @brief Configuration for the packing solver
 */
struct SolverConfig {
    /// Available bin types to use for packing
    std::vector<BinType> bin_types;

    /// Whether multiple bins can be used
    bool allow_multiple_bins = true;

    /// Maximum time to spend on solving
    std::chrono::milliseconds timeout{std::chrono::minutes(5)};

    /// Number of threads to use (0 = auto-detect)
    unsigned int thread_count = 0;

    /// Get effective thread count
    [[nodiscard]] unsigned int effective_thread_count() const noexcept {
        if (thread_count == 0) {
            unsigned int hw = std::thread::hardware_concurrency();
            return hw > 0 ? hw : 1;
        }
        return thread_count;
    }
};

}  // namespace bp3d

#endif  // BP3D_CONFIG_HPP
