/**
 * @file free_space.hpp
 * @brief Free space tracking data structures for bin packing algorithms
 */

#ifndef BP3D_INTERNAL_FREE_SPACE_HPP
#define BP3D_INTERNAL_FREE_SPACE_HPP

#include "bp3d/rotation.hpp"
#include "bp3d/types.hpp"
#include "geometry.hpp"

#include <algorithm>
#include <vector>

namespace bp3d::internal {

/**
 * @brief A free rectangular space within a bin
 */
struct FreeSpace {
    Vector3 position;       ///< Bottom-left-back corner
    Dimensions dimensions;  ///< Size of the free space

    /// Get volume of this free space
    [[nodiscard]] double volume() const { return dimensions.volume(); }

    /// Convert to AABB
    [[nodiscard]] AABB to_aabb() const { return AABB::from_pos_dims(position, dimensions); }

    /// Check if dimensions fit in this space
    [[nodiscard]] bool fits(const Dimensions& dims) const {
        return dims.width <= dimensions.width && dims.height <= dimensions.height &&
               dims.depth <= dimensions.depth;
    }
};

/**
 * @brief An extreme point (corner candidate for placement)
 */
struct ExtremePoint {
    Vector3 position;           ///< Position of the extreme point
    Dimensions residual_space;  ///< Available space in each direction

    /// Check if dimensions fit at this point
    [[nodiscard]] bool fits(const Dimensions& dims) const {
        return dims.width <= residual_space.width && dims.height <= residual_space.height &&
               dims.depth <= residual_space.depth;
    }
};

/**
 * @brief Manages free spaces using maximal rectangles approach
 *
 * When an item is placed, the affected free spaces are split into
 * smaller maximal free rectangles.
 */
class MaximalSpaceManager {
public:
    explicit MaximalSpaceManager(const Dimensions& bin_dims);

    /// Add initial free space (the entire bin)
    void initialize();

    /// Get all current free spaces
    [[nodiscard]] const std::vector<FreeSpace>& free_spaces() const { return spaces_; }

    /// Find best free space for given dimensions
    [[nodiscard]] std::optional<std::size_t> find_best_fit(const Dimensions& dims) const;

    /// Place an item at the given position, update free spaces
    void place_item(const Vector3& position, const Dimensions& dims);

    /// Get remaining free volume
    [[nodiscard]] double free_volume() const;

private:
    Dimensions bin_dims_;
    std::vector<FreeSpace> spaces_;

    /// Split a free space when an item is placed inside it
    void split_space(const FreeSpace& space, const Vector3& pos, const Dimensions& dims);

    /// Remove spaces that are fully contained within others
    void remove_redundant_spaces();
};

/**
 * @brief Manages extreme points for online packing
 *
 * Extreme points are corners where new items can potentially be placed.
 */
class ExtremePointManager {
public:
    explicit ExtremePointManager(const Dimensions& bin_dims);

    /// Initialize with the origin point
    void initialize();

    /// Get all current extreme points
    [[nodiscard]] const std::vector<ExtremePoint>& points() const { return points_; }

    /// Update points after placing an item
    void update(const Vector3& position, const Dimensions& dims);

    /// Calculate residual space for a point
    void update_residual_space(ExtremePoint& point, std::span<const Placement> placements);

private:
    Dimensions bin_dims_;
    std::vector<ExtremePoint> points_;
};

/**
 * @brief Shelf/layer for shelf-based packing
 */
struct Shelf {
    double y_position;             ///< Vertical position of shelf floor
    double height;                 ///< Height of the shelf
    double used_width;             ///< Width used so far
    double used_depth;             ///< Depth used so far
    std::vector<Placement> items;  ///< Items on this shelf
};

/**
 * @brief Manages shelves for shelf-based packing
 */
class ShelfManager {
public:
    explicit ShelfManager(const Dimensions& bin_dims);

    /// Get current shelves
    [[nodiscard]] const std::vector<Shelf>& shelves() const { return shelves_; }

    /// Try to place an item on existing shelves
    [[nodiscard]] std::optional<Vector3> find_position(const Dimensions& dims);

    /// Create a new shelf with the given height
    bool create_shelf(double height);

    /// Place item at position (updates shelf state)
    void place_item(const Vector3& position, const Dimensions& dims, const Placement& placement);

private:
    Dimensions bin_dims_;
    std::vector<Shelf> shelves_;
};

}  // namespace bp3d::internal

#endif  // BP3D_INTERNAL_FREE_SPACE_HPP
