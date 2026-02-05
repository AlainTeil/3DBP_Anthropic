/**
 * @file collision.cpp
 * @brief CollisionValidator implementation
 */

#include "bp3d/constraints/collision.hpp"

namespace bp3d {

bool placements_overlap(const Placement& a, const Placement& b) {
    // AABB overlap test: boxes overlap if they overlap on all three axes
    const auto a_max = a.max_corner();
    const auto b_max = b.max_corner();

    // Check for separation on each axis
    // If separated on any axis, no overlap
    if (a.position.x >= b_max.x || b.position.x >= a_max.x) {
        return false;
    }
    if (a.position.y >= b_max.y || b.position.y >= a_max.y) {
        return false;
    }
    if (a.position.z >= b_max.z || b.position.z >= a_max.z) {
        return false;
    }

    return true;
}

bool CollisionValidator::can_place(const Item& /*item*/, const Placement& proposed,
                                   std::span<const Placement> existing, const BinType& bin,
                                   const ItemRegistry& /*registry*/) const {
    // First check if the placement is within bin bounds
    const auto max_corner = proposed.max_corner();
    if (proposed.position.x < 0 || proposed.position.y < 0 || proposed.position.z < 0) {
        return false;
    }
    if (max_corner.x > bin.dimensions.width || max_corner.y > bin.dimensions.height ||
        max_corner.z > bin.dimensions.depth) {
        return false;
    }

    // Check for collisions with existing placements
    for (const auto& existing_placement : existing) {
        if (placements_overlap(proposed, existing_placement)) {
            return false;
        }
    }

    return true;
}

}  // namespace bp3d
