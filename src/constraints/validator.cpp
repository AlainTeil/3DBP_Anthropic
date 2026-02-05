/**
 * @file validator.cpp
 * @brief CompositeValidator implementation
 */

#include "bp3d/constraints/validator.hpp"

#include "bp3d/constraints/collision.hpp"
#include "bp3d/constraints/stacking.hpp"
#include "bp3d/constraints/weight.hpp"

namespace bp3d {

void CompositeValidator::add(std::unique_ptr<IConstraintValidator> validator) {
    if (validator) {
        validators_.push_back(std::move(validator));
    }
}

bool CompositeValidator::can_place(const Item& item, const Placement& proposed,
                                   std::span<const Placement> existing, const BinType& bin,
                                   const ItemRegistry& registry) const {
    for (const auto& validator : validators_) {
        if (!validator->can_place(item, proposed, existing, bin, registry)) {
            return false;
        }
    }
    return true;
}

std::unique_ptr<CompositeValidator> create_default_validator() {
    auto composite = std::make_unique<CompositeValidator>();
    composite->add(std::make_unique<CollisionValidator>());
    composite->add(std::make_unique<StackingValidator>(true));
    // Note: WeightValidator requires item_weights to be set externally
    composite->add(std::make_unique<WeightValidator>());
    return composite;
}

}  // namespace bp3d
