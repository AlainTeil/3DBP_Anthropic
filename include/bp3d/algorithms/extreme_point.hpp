/**
 * @file extreme_point.hpp
 * @brief Extreme Point algorithm for online 3D bin packing
 */

#ifndef BP3D_ALGORITHMS_EXTREME_POINT_HPP
#define BP3D_ALGORITHMS_EXTREME_POINT_HPP

#include "bp3d/solver.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <vector>

namespace bp3d {

namespace internal {
class PlacementEngine;
}  // namespace internal

/**
 * @brief Point selection heuristic for extreme point algorithm
 */
enum class ExtremePointHeuristic {
    BottomLeft,         ///< Prefer the lowest point: smallest Y, then X, then Z
    BottomLeftFill,     ///< Prefer low points nearest the origin corner (fill gaps first)
    TouchingPerimeter,  ///< Prefer placements with the most face contact (bin walls + neighbours)
    MinWastedSpace      ///< Prefer placements that keep the occupied bounding box smallest
};

/**
 * @brief Extreme Point algorithm for 3D bin packing
 *
 * The extreme point algorithm tracks "extreme points" - corners where
 * new items can be placed. After each placement, new extreme points
 * are generated at the corners of the placed item.
 *
 * This is an online algorithm that can pack items one at a time.
 */
class ExtremePointSolver : public IOnlineSolver {
public:
    /**
     * @brief Constructor with configurable heuristic
     *
     * @param heuristic Point selection heuristic
     */
    explicit ExtremePointSolver(
        ExtremePointHeuristic heuristic = ExtremePointHeuristic::BottomLeft);

    ~ExtremePointSolver() override;

    // IOnlineSolver interface
    void reset(const SolverConfig& config) override;
    [[nodiscard]] std::optional<Placement> pack_one(const Item& item) override;
    [[nodiscard]] PackingResult finalize() override;

    // ISolver interface
    [[nodiscard]] PackingResult solve(std::span<const Item> items,
                                      const SolverConfig& config) override;

    [[nodiscard]] std::string_view name() const override { return "ExtremePoint"; }

private:
    ExtremePointHeuristic heuristic_;
    SolverConfig config_;

    /// Shared engine: owns bin bookkeeping, the item registry, the spatial
    /// index and the constraint pipeline (collision, support, do-not-stack,
    /// weight limits).
    std::unique_ptr<internal::PlacementEngine> engine_;

    /// Per-bin incremental extreme-point set (the only algorithm-specific
    /// state), kept in a vector parallel to engine_->bins().
    std::vector<std::vector<Vector3>> extreme_points_;

    std::vector<std::string> unpacked_;
    std::chrono::steady_clock::time_point start_time_;

    /// Evaluate every extreme point and allowed rotation in the bin at
    /// @p bin_index and return the placement that best satisfies the configured
    /// heuristic, or nullopt if the item cannot be placed in this bin.
    [[nodiscard]] std::optional<Placement> select_best_placement(std::size_t bin_index,
                                                                 const Item& item);

    /// Heuristic ranking key for a candidate placement (lexicographically
    /// smaller is better). The key contents differ per @ref ExtremePointHeuristic.
    [[nodiscard]] std::array<double, 3> score_placement(std::size_t bin_index,
                                                        const Placement& placement);

    /// Add a new bin. If @p item is non-null, the cheapest configured bin type
    /// that can accommodate the item (dimensions in some allowed rotation and
    /// weight) is chosen; otherwise the first configured bin type is used.
    void add_bin(const Item* item = nullptr);

    /// Select the bin type to open for an item: the lowest-cost configured type
    /// that can fit the item, falling back to the first type if none fit.
    [[nodiscard]] const BinType& select_bin_type(const Item* item) const;

    /// Update extreme points after placement
    void update_extreme_points(std::size_t bin_index, const Placement& placement);
};

}  // namespace bp3d

#endif  // BP3D_ALGORITHMS_EXTREME_POINT_HPP
