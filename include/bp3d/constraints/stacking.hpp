/**
 * @file stacking.hpp
 * @brief Stacking constraint validators
 */

#ifndef BP3D_CONSTRAINTS_STACKING_HPP
#define BP3D_CONSTRAINTS_STACKING_HPP

#include "bp3d/constraints/validator.hpp"

namespace bp3d {

/**
 * @brief Validates stacking constraints
 *
 * Ensures that items marked as non-stackable don't have anything placed on
 * top of them, and items are properly supported from below.
 */
class StackingValidator : public IConstraintValidator {
public:
    /**
     * @brief Constructor
     *
     * @param require_support If true, items must be supported from below
     *                        (either by bin floor or another item)
     */
    explicit StackingValidator(bool require_support = true);

    [[nodiscard]] bool can_place(const Item& item, const Placement& proposed,
                                 std::span<const Placement> existing, const BinType& bin,
                                 const ItemRegistry& registry = {}) const override;

    [[nodiscard]] std::string_view name() const override { return "Stacking"; }

private:
    bool require_support_;
};

/**
 * @brief Check if placement B is on top of placement A
 *
 * B is considered "on top of" A if B's bottom face overlaps with A's top face.
 *
 * @param below Placement that might be below
 * @param above Placement that might be above
 * @param tolerance Vertical gap tolerance
 * @return true if above is resting on below
 */
[[nodiscard]] bool is_on_top_of(const Placement& below, const Placement& above,
                                double tolerance = 1e-6);

/**
 * @brief Check if a placement is supported from below
 *
 * @param proposed Placement to check
 * @param existing Existing placements
 * @param bin_floor_y Y coordinate of the bin floor (usually 0)
 * @param tolerance Tolerance for support detection
 * @return true if the placement is properly supported
 */
[[nodiscard]] bool is_supported(const Placement& proposed, std::span<const Placement> existing,
                                double bin_floor_y = 0.0, double tolerance = 1e-6);

}  // namespace bp3d

#endif  // BP3D_CONSTRAINTS_STACKING_HPP
