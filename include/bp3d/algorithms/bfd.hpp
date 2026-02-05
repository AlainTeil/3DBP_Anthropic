/**
 * @file bfd.hpp
 * @brief Best Fit Decreasing (BFD) algorithm
 */

#ifndef BP3D_ALGORITHMS_BFD_HPP
#define BP3D_ALGORITHMS_BFD_HPP

#include "bp3d/solver.hpp"

namespace bp3d {

/**
 * @brief Best Fit Decreasing algorithm for 3D bin packing
 *
 * BFD sorts items by volume in decreasing order, then places each item
 * in the bin where it fits with minimum wasted space. If no existing
 * bin can accommodate the item, a new bin is opened.
 *
 * This typically achieves better space utilization than FFD at the
 * cost of more computation per placement decision.
 *
 * This is an offline algorithm that processes all items at once.
 */
class BestFitDecreasing : public ISolver {
public:
    [[nodiscard]] PackingResult solve(std::span<const Item> items,
                                      const SolverConfig& config) override;

    [[nodiscard]] std::string_view name() const override { return "BestFitDecreasing"; }

    [[nodiscard]] bool supports_online() const override { return false; }
};

}  // namespace bp3d

#endif  // BP3D_ALGORITHMS_BFD_HPP
