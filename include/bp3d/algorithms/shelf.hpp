/**
 * @file shelf.hpp
 * @brief Shelf-based bin packing algorithm
 */

#ifndef BP3D_ALGORITHMS_SHELF_HPP
#define BP3D_ALGORITHMS_SHELF_HPP

#include "bp3d/solver.hpp"

#include <vector>

namespace bp3d {

/**
 * @brief Shelf selection heuristic
 */
enum class ShelfHeuristic {
    NextFit,       ///< Only try the current (most recent) shelf
    FirstFit,      ///< Try shelves in order, use first that fits
    BestWidthFit,  ///< Use shelf with closest matching unused width
    BestHeightFit  ///< Use shelf with closest matching height
};

/**
 * @brief Shelf (level) algorithm for 3D bin packing
 *
 * The shelf algorithm organizes items into horizontal layers (shelves).
 * Each shelf has a fixed height determined by the first item placed on it.
 * Items are placed left-to-right on each shelf.
 *
 * Simple, fast, and works well for items of similar heights.
 *
 * This is an online algorithm that can pack items one at a time.
 */
class ShelfSolver : public IOnlineSolver {
public:
    /**
     * @brief Constructor with configurable heuristic
     *
     * @param heuristic Shelf selection heuristic
     */
    explicit ShelfSolver(ShelfHeuristic heuristic = ShelfHeuristic::FirstFit);

    // IOnlineSolver interface
    void reset(const SolverConfig& config) override;
    [[nodiscard]] std::optional<Placement> pack_one(const Item& item) override;
    [[nodiscard]] PackingResult finalize() override;

    // ISolver interface
    [[nodiscard]] PackingResult solve(std::span<const Item> items,
                                      const SolverConfig& config) override;

    [[nodiscard]] std::string_view name() const override { return "Shelf"; }

private:
    ShelfHeuristic heuristic_;
    SolverConfig config_;

    /// Shelf within a bin
    struct Shelf {
        double y_position;
        double height;
        double used_width;
    };

    /// Per-bin state
    struct BinState {
        BinType type;
        int index;
        std::vector<Placement> placements;
        std::vector<Shelf> shelves;
        double used_weight;
    };

    std::vector<BinState> bins_;
    std::vector<std::string> unpacked_;
    std::chrono::steady_clock::time_point start_time_;

    /// Try to place item on a shelf
    [[nodiscard]] std::optional<Placement>
    try_place_on_shelf(BinState& bin, Shelf& shelf, const Item& item, const Dimensions& rotated);

    /// Try to create a new shelf and place item
    [[nodiscard]] std::optional<Placement> try_new_shelf(BinState& bin, const Item& item,
                                                         const Dimensions& rotated);

    /// Add a new bin
    void add_bin();
};

}  // namespace bp3d

#endif  // BP3D_ALGORITHMS_SHELF_HPP
