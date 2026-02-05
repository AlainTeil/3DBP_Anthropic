/**
 * @file weight.hpp
 * @brief Weight-based constraint validators
 */

#ifndef BP3D_CONSTRAINTS_WEIGHT_HPP
#define BP3D_CONSTRAINTS_WEIGHT_HPP

#include "bp3d/constraints/validator.hpp"

#include <unordered_map>

namespace bp3d {

/**
 * @brief Validates weight constraints
 *
 * Checks:
 * - Total bin weight doesn't exceed maximum
 * - Individual items don't exceed their max stack weight limit
 */
class WeightValidator : public IConstraintValidator {
public:
    /**
     * @brief Constructor
     *
     * @param item_weights Map from item_id to item weight (for looking up weights
     *                     of existing placements)
     */
    explicit WeightValidator(std::unordered_map<std::string, double> item_weights = {});

    /**
     * @brief Set the item weights map
     */
    void set_item_weights(std::unordered_map<std::string, double> weights);

    /**
     * @brief Add or update weight for an item
     */
    void add_item_weight(const std::string& item_id, double weight);

    [[nodiscard]] bool can_place(const Item& item, const Placement& proposed,
                                 std::span<const Placement> existing, const BinType& bin,
                                 const ItemRegistry& registry = {}) const override;

    [[nodiscard]] std::string_view name() const override { return "Weight"; }

private:
    std::unordered_map<std::string, double> item_weights_;
};

/**
 * @brief Calculate total weight of items stacked on top of a given item
 *
 * @param item_placement The item to check
 * @param all_placements All placements in the bin
 * @param item_weights Map from item_id to weight
 * @return Total weight stacked on the item
 */
[[nodiscard]] double
calculate_stacked_weight(const Placement& item_placement, std::span<const Placement> all_placements,
                         const std::unordered_map<std::string, double>& item_weights);

}  // namespace bp3d

#endif  // BP3D_CONSTRAINTS_WEIGHT_HPP
