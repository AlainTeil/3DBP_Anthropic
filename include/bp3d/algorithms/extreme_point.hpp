/**
 * @file extreme_point.hpp
 * @brief Extreme Point algorithm for online 3D bin packing
 */

#ifndef BP3D_ALGORITHMS_EXTREME_POINT_HPP
#define BP3D_ALGORITHMS_EXTREME_POINT_HPP

#include "bp3d/solver.hpp"

#include <vector>

namespace bp3d {

/**
 * @brief Point selection heuristic for extreme point algorithm
 */
enum class ExtremePointHeuristic {
    BottomLeft,         ///< Prefer points with lowest Y, then X, then Z
    BottomLeftFill,     ///< Like BottomLeft but also considers fill ratio
    TouchingPerimeter,  ///< Prefer points that maximize touching edges
    MinWastedSpace      ///< Prefer points that minimize wasted space
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

    // Per-bin state
    struct BinState {
        BinType type;
        int index;
        std::vector<Placement> placements;
        std::vector<Vector3> extreme_points;
        double used_weight;
    };

    std::vector<BinState> bins_;
    std::vector<std::string> unpacked_;
    std::chrono::steady_clock::time_point start_time_;

    /// Try to place item in a specific bin at a specific point
    [[nodiscard]] std::optional<Placement> try_place_at(BinState& bin, const Item& item,
                                                        const Vector3& point);

    /// Add a new bin
    void add_bin();

    /// Update extreme points after placement
    void update_extreme_points(BinState& bin, const Placement& placement);
};

}  // namespace bp3d

#endif  // BP3D_ALGORITHMS_EXTREME_POINT_HPP
