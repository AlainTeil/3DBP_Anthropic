/**
 * @file spatial_grid.hpp
 * @brief Uniform spatial-hash grid for accelerating neighbour queries.
 *
 * Bin packing repeatedly asks "which already-placed boxes are near this
 * candidate position?" to test collisions and support. A linear scan makes
 * that O(n) per query and O(n^2)-O(n^3) overall. The SpatialGrid buckets each
 * box into fixed-size cubic cells so a neighbour query only inspects boxes in
 * the cells overlapping the query region.
 */

#ifndef BP3D_INTERNAL_SPATIAL_GRID_HPP
#define BP3D_INTERNAL_SPATIAL_GRID_HPP

#include "internal/geometry.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace bp3d::internal {

/**
 * @brief Uniform spatial-hash grid over axis-aligned boxes within one bin.
 *
 * Boxes are referenced by an opaque @c index (typically their position in the
 * bin's placement vector). A box is registered in every cell its AABB overlaps;
 * a query returns the indices of all boxes sharing a cell with the query box.
 * Queries may return false positives (boxes in the same cell that do not
 * actually overlap) but never false negatives, so callers must still run an
 * exact test on the returned candidates.
 */
class SpatialGrid {
public:
    /**
     * @param cell_size Edge length of each cubic cell. Values <= 0 are clamped
     *        to a small positive default so the grid stays usable.
     */
    explicit SpatialGrid(double cell_size);

    /// Register box @p index occupying @p box.
    void insert(std::size_t index, const AABB& box);

    /**
     * @brief Append candidate neighbour indices for @p box to @p out.
     *
     * Each index is appended at most once. @p out is not cleared first, letting
     * callers reuse a scratch buffer.
     */
    void query(const AABB& box, std::vector<std::size_t>& out) const;

    /// Remove every registered box (keeps the configured cell size).
    void clear() noexcept;

    /// Number of distinct boxes currently registered.
    [[nodiscard]] std::size_t size() const noexcept { return count_; }

    /// Configured cell edge length.
    [[nodiscard]] double cell_size() const noexcept { return cell_size_; }

private:
    struct CellKey {
        int x;
        int y;
        int z;
        bool operator==(const CellKey& other) const noexcept = default;
    };

    struct CellKeyHash {
        [[nodiscard]] std::size_t operator()(const CellKey& key) const noexcept;
    };

    [[nodiscard]] int cell_index(double coord) const noexcept;

    double cell_size_;
    std::size_t count_ = 0;
    std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> cells_;

    // Bounding range of populated cell coordinates. Queries are clamped to this
    // range so a candidate box far larger than the occupied region cannot make
    // the cell-iteration loop span an astronomical number of empty cells.
    bool has_cells_ = false;
    int min_cell_x_ = 0;
    int min_cell_y_ = 0;
    int min_cell_z_ = 0;
    int max_cell_x_ = 0;
    int max_cell_y_ = 0;
    int max_cell_z_ = 0;

    // Epoch-stamped scratch used to deduplicate query results without
    // allocating per call. seen_[i] == epoch_ means index i is already in out.
    mutable std::vector<std::uint32_t> seen_;
    mutable std::uint32_t epoch_ = 0;
};

}  // namespace bp3d::internal

#endif  // BP3D_INTERNAL_SPATIAL_GRID_HPP
