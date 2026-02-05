/**
 * @file weight.cpp
 * @brief WeightValidator implementation
 */

#include "bp3d/constraints/weight.hpp"

#include "bp3d/constraints/stacking.hpp"

namespace bp3d {

WeightValidator::WeightValidator(std::unordered_map<std::string, double> item_weights)
    : item_weights_(std::move(item_weights)) {}

void WeightValidator::set_item_weights(std::unordered_map<std::string, double> weights) {
    item_weights_ = std::move(weights);
}

void WeightValidator::add_item_weight(const std::string& item_id, double weight) {
    item_weights_[item_id] = weight;
}

double calculate_stacked_weight(const Placement& item_placement,
                                std::span<const Placement> all_placements,
                                const std::unordered_map<std::string, double>& item_weights) {
    double total_weight = 0.0;

    for (const auto& other : all_placements) {
        // Skip self
        if (&other == &item_placement) {
            continue;
        }

        // Check if 'other' is on top of item_placement
        if (is_on_top_of(item_placement, other, 1e-6)) {
            auto it = item_weights.find(other.item_id);
            if (it != item_weights.end()) {
                total_weight += it->second;
            }
        }
    }

    return total_weight;
}

bool WeightValidator::can_place(const Item& item, const Placement& proposed,
                                std::span<const Placement> existing, const BinType& bin,
                                const ItemRegistry& registry) const {
    // Calculate current total weight in bin
    double current_weight = 0.0;
    for (const auto& placement : existing) {
        // First try the registry, then fall back to item_weights_
        auto reg_it = registry.find(placement.item_id);
        if (reg_it != registry.end()) {
            current_weight += reg_it->second.weight;
        } else {
            auto it = item_weights_.find(placement.item_id);
            if (it != item_weights_.end()) {
                current_weight += it->second;
            }
        }
    }

    // Check if adding this item exceeds bin weight limit
    if (current_weight + item.weight > bin.max_weight) {
        return false;
    }

    // Check if placing this item on top of any existing item would exceed
    // that item's max stack weight
    for (const auto& below : existing) {
        if (is_on_top_of(below, proposed, 1e-6)) {
            // Look up the item in the registry to check max_stack_weight
            auto it = registry.find(below.item_id);
            if (it != registry.end()) {
                const Item& below_item = it->second;
                if (below_item.constraints.max_stack_weight > 0 &&
                    item.weight > below_item.constraints.max_stack_weight) {
                    return false;  // Would exceed max stack weight
                }
            }
        }
    }

    return true;
}

}  // namespace bp3d
