/**
 * @file stacking.cpp
 * @brief StackingValidator implementation
 */

#include "bp3d/constraints/stacking.hpp"

#include "bp3d/constraints/validator.hpp"
#include "bp3d/result.hpp"
#include "bp3d/types.hpp"

#include <algorithm>
#include <cmath>
#include <span>

namespace bp3d {

namespace {

// Check if two rectangles on the XZ plane overlap
bool rectangles_overlap_xz(const Placement& a, const Placement& b) {
    const auto a_max = a.max_corner();
    const auto b_max = b.max_corner();

    if (a.position.x >= b_max.x || b.position.x >= a_max.x) {
        return false;
    }
    if (a.position.z >= b_max.z || b.position.z >= a_max.z) {
        return false;
    }
    return true;
}

}  // namespace

StackingValidator::StackingValidator(bool require_support) : require_support_(require_support) {}

bool is_on_top_of(const Placement& below, const Placement& above, double tolerance) {
    // Check if the bottom of 'above' is at the top of 'below'
    const double below_top = below.position.y + below.rotated_dimensions.height;
    const double above_bottom = above.position.y;

    if (std::abs(above_bottom - below_top) > tolerance) {
        return false;
    }

    // Check XZ overlap
    return rectangles_overlap_xz(below, above);
}

bool is_supported(const Placement& proposed, std::span<const Placement> existing,
                  double bin_floor_y, double tolerance) {
    // If on the floor, it's supported
    if (std::abs(proposed.position.y - bin_floor_y) <= tolerance) {
        return true;
    }

    // Otherwise, check if any existing placement supports this one
    return std::ranges::any_of(existing, [&](const Placement& placement) {
        return is_on_top_of(placement, proposed, tolerance);
    });
}

bool StackingValidator::can_place(const Item& item, const Placement& proposed,
                                  std::span<const Placement> existing, const BinType& /*bin*/,
                                  const ItemRegistry& registry) const {
    // Check if we require support and this item is not supported
    if (require_support_ && !is_supported(proposed, existing, 0.0, kContactTolerance)) {
        return false;
    }

    // If this item is non-stackable (nothing can be placed on top),
    // we can't prevent future placements from here.
    // This constraint is checked when placing OTHER items on top.

    // Check if we're trying to place on top of a non-stackable item
    for (const auto& below : existing) {
        if (is_on_top_of(below, proposed, kContactTolerance)) {
            // This placement would be on top of 'below'
            // Look up the item in the registry to check stackable
            auto it = registry.find(below.item_id);
            if (it != registry.end()) {
                const Item& below_item = it->second;
                if (!below_item.constraints.stackable) {
                    return false;  // Cannot place anything on a non-stackable item
                }
            }
        }
    }

    // Additional check: if the item being placed is marked non-stackable,
    // we don't restrict its placement - we just can't put things on it later
    (void)item;  // Mark as intentionally unused for this check

    return true;
}

}  // namespace bp3d
