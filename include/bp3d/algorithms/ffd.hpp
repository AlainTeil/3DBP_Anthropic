/**
 * @file ffd.hpp
 * @brief First Fit Decreasing (FFD) algorithm
 */

#ifndef BP3D_ALGORITHMS_FFD_HPP
#define BP3D_ALGORITHMS_FFD_HPP

#include "bp3d/solver.hpp"

namespace bp3d {

/**
 * @brief First Fit Decreasing algorithm for 3D bin packing
 *
 * FFD sorts items by volume in decreasing order, then places each item
 * in the first bin where it fits. If no existing bin can accommodate
 * the item, a new bin is opened.
 *
 * This is an offline algorithm that processes all items at once.
 */
class FirstFitDecreasing : public ISolver {
public:
    [[nodiscard]] PackingResult solve(std::span<const Item> items,
                                      const SolverConfig& config) override;

    [[nodiscard]] std::string_view name() const override { return "FirstFitDecreasing"; }

    [[nodiscard]] bool supports_online() const override { return false; }
};

}  // namespace bp3d

#endif  // BP3D_ALGORITHMS_FFD_HPP
