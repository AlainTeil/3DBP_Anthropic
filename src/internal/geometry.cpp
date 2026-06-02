/**
 * @file geometry.cpp
 * @brief Geometry utility implementations
 */

#include "geometry.hpp"

#include "bp3d/result.hpp"
#include "bp3d/types.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <span>

namespace bp3d::internal {

bool AABB::overlaps(const AABB& other) const {
    // Separating axis test - if separated on any axis, no overlap
    if (max.x <= other.min.x || other.max.x <= min.x) {
        return false;
    }
    if (max.y <= other.min.y || other.max.y <= min.y) {
        return false;
    }
    if (max.z <= other.min.z || other.max.z <= min.z) {
        return false;
    }
    return true;
}

bool AABB::contains(const Vector3& point) const {
    return point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y &&
           point.z >= min.z && point.z <= max.z;
}

bool AABB::contains(const AABB& other) const {
    return other.min.x >= min.x && other.max.x <= max.x && other.min.y >= min.y &&
           other.max.y <= max.y && other.min.z >= min.z && other.max.z <= max.z;
}

bool fits_in(const Dimensions& box, const Dimensions& space) {
    return box.width <= space.width && box.height <= space.height && box.depth <= space.depth;
}

std::optional<AABB> intersection(const AABB& a, const AABB& b) {
    double min_x = std::max(a.min.x, b.min.x);
    double max_x = std::min(a.max.x, b.max.x);
    if (min_x >= max_x) {
        return std::nullopt;
    }

    double min_y = std::max(a.min.y, b.min.y);
    double max_y = std::min(a.max.y, b.max.y);
    if (min_y >= max_y) {
        return std::nullopt;
    }

    double min_z = std::max(a.min.z, b.min.z);
    double max_z = std::min(a.max.z, b.max.z);
    if (min_z >= max_z) {
        return std::nullopt;
    }

    return AABB{{min_x, min_y, min_z}, {max_x, max_y, max_z}};
}

double support_ratio(const Placement& proposed, std::span<const Placement> existing,
                     double floor_y) {
    const double tolerance = kContactTolerance;

    // If on the floor, fully supported
    if (std::abs(proposed.position.y - floor_y) <= tolerance) {
        return 1.0;
    }

    // Calculate area of the bottom of proposed
    double const total_area = proposed.rotated_dimensions.width * proposed.rotated_dimensions.depth;
    if (total_area <= 0) {
        return 0.0;
    }

    // Find all placements that could support this one
    double supported_area = 0.0;
    for (const auto& below : existing) {
        // Check if top of below is at bottom of proposed
        double const below_top = below.position.y + below.rotated_dimensions.height;
        if (std::abs(below_top - proposed.position.y) > tolerance) {
            continue;
        }

        // Calculate XZ overlap area
        double const overlap_x_min = std::max(proposed.position.x, below.position.x);
        double const overlap_x_max =
            std::min(proposed.position.x + proposed.rotated_dimensions.width,
                     below.position.x + below.rotated_dimensions.width);
        double const overlap_z_min = std::max(proposed.position.z, below.position.z);
        double const overlap_z_max =
            std::min(proposed.position.z + proposed.rotated_dimensions.depth,
                     below.position.z + below.rotated_dimensions.depth);

        if (overlap_x_max > overlap_x_min && overlap_z_max > overlap_z_min) {
            double const overlap_area =
                (overlap_x_max - overlap_x_min) * (overlap_z_max - overlap_z_min);
            supported_area += overlap_area;
        }
    }

    // Clamp to 1.0 (in case of overlapping supports)
    return std::min(supported_area / total_area, 1.0);
}

}  // namespace bp3d::internal
