/**
 * @file geometry.hpp
 * @brief Internal geometry utilities
 */

#ifndef BP3D_INTERNAL_GEOMETRY_HPP
#define BP3D_INTERNAL_GEOMETRY_HPP

#include "bp3d/result.hpp"
#include "bp3d/types.hpp"

namespace bp3d::internal {

/**
 * @brief Axis-Aligned Bounding Box
 */
struct AABB {
    Vector3 min;  ///< Minimum corner
    Vector3 max;  ///< Maximum corner

    /// Create AABB from placement
    static AABB from_placement(const Placement& p) { return {p.position, p.max_corner()}; }

    /// Create AABB from position and dimensions
    static AABB from_pos_dims(const Vector3& pos, const Dimensions& dims) {
        return {pos, {pos.x + dims.width, pos.y + dims.height, pos.z + dims.depth}};
    }

    /// Get dimensions
    [[nodiscard]] Dimensions dimensions() const {
        return {max.x - min.x, max.y - min.y, max.z - min.z};
    }

    /// Get volume
    [[nodiscard]] double volume() const { return dimensions().volume(); }

    /// Check if this AABB overlaps with another
    [[nodiscard]] bool overlaps(const AABB& other) const;

    /// Check if this AABB contains a point
    [[nodiscard]] bool contains(const Vector3& point) const;

    /// Check if this AABB fully contains another
    [[nodiscard]] bool contains(const AABB& other) const;
};

/**
 * @brief Check if a box fits within given dimensions
 */
[[nodiscard]] bool fits_in(const Dimensions& box, const Dimensions& space);

/**
 * @brief Compute intersection of two AABBs
 *
 * @return The intersection AABB, or nullopt if no intersection
 */
[[nodiscard]] std::optional<AABB> intersection(const AABB& a, const AABB& b);

/**
 * @brief Calculate the support area of a placement on top of existing
 * placements
 *
 * @param proposed The proposed placement
 * @param existing Existing placements
 * @param floor_y Y coordinate of the floor
 * @return Ratio of supported area (0-1)
 */
[[nodiscard]] double support_ratio(const Placement& proposed, std::span<const Placement> existing,
                                   double floor_y = 0.0);

}  // namespace bp3d::internal

#endif  // BP3D_INTERNAL_GEOMETRY_HPP
