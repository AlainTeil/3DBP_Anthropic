/**
 * @file guillotine.hpp
 * @brief Guillotine-based bin packing algorithm
 */

#ifndef BP3D_ALGORITHMS_GUILLOTINE_HPP
#define BP3D_ALGORITHMS_GUILLOTINE_HPP

#include "bp3d/solver.hpp"

namespace bp3d {

/**
 * @brief Split rules for guillotine algorithm
 */
enum class GuillotineSplitRule {
    ShorterLeftoverAxis,  ///< Split along the axis with shorter leftover
    LongerLeftoverAxis,   ///< Split along the axis with longer leftover
    MinimizeArea,         ///< Split to minimize wasted area
    MaximizeArea          ///< Split to maximize larger piece
};

/**
 * @brief Selection rules for guillotine algorithm
 */
enum class GuillotineFreeRectChoice {
    BestShortSideFit,  ///< BSSF: Best fit by short side
    BestLongSideFit,   ///< BLSF: Best fit by long side
    BestAreaFit,       ///< BAF: Best fit by area
    WorstAreaFit       ///< WAF: Worst fit by area (saves larger spaces)
};

/**
 * @brief Guillotine algorithm for 3D bin packing
 *
 * The guillotine algorithm maintains a list of free rectangles.
 * When an item is placed, the containing free rectangle is split
 * using guillotine cuts (axis-aligned splits that go edge-to-edge).
 *
 * This is an offline algorithm that processes all items at once.
 */
class GuillotineSolver : public ISolver {
public:
    /**
     * @brief Constructor with configurable rules
     *
     * @param split_rule How to split free rectangles
     * @param choice_rule How to choose among free rectangles
     */
    explicit GuillotineSolver(
        GuillotineSplitRule split_rule = GuillotineSplitRule::ShorterLeftoverAxis,
        GuillotineFreeRectChoice choice_rule = GuillotineFreeRectChoice::BestShortSideFit);

    [[nodiscard]] PackingResult solve(std::span<const Item> items,
                                      const SolverConfig& config) override;

    [[nodiscard]] std::string_view name() const override { return "Guillotine"; }

    [[nodiscard]] bool supports_online() const override { return false; }

private:
    GuillotineSplitRule split_rule_;
    GuillotineFreeRectChoice choice_rule_;
};

}  // namespace bp3d

#endif  // BP3D_ALGORITHMS_GUILLOTINE_HPP
