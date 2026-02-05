/**
 * @file collision.hpp
 * @brief Collision detection validator
 */

#ifndef BP3D_CONSTRAINTS_COLLISION_HPP
#define BP3D_CONSTRAINTS_COLLISION_HPP

#include "bp3d/constraints/validator.hpp"

namespace bp3d {

/**
 * @brief Validates that items don't overlap with each other
 *
 * Uses AABB (Axis-Aligned Bounding Box) intersection test.
 */
class CollisionValidator : public IConstraintValidator {
public:
    [[nodiscard]] bool can_place(const Item& item, const Placement& proposed,
                                 std::span<const Placement> existing, const BinType& bin,
                                 const ItemRegistry& registry = {}) const override;

    [[nodiscard]] std::string_view name() const override { return "Collision"; }
};

/**
 * @brief Check if two placements overlap
 *
 * @param a First placement
 * @param b Second placement
 * @return true if the placements overlap
 */
[[nodiscard]] bool placements_overlap(const Placement& a, const Placement& b);

}  // namespace bp3d

#endif  // BP3D_CONSTRAINTS_COLLISION_HPP
