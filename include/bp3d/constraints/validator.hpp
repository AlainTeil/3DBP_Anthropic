/**
 * @file validator.hpp
 * @brief Constraint validation interfaces
 */

#ifndef BP3D_CONSTRAINTS_VALIDATOR_HPP
#define BP3D_CONSTRAINTS_VALIDATOR_HPP

#include "bp3d/result.hpp"
#include "bp3d/types.hpp"

#include <memory>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace bp3d {

/**
 * @brief Registry mapping item IDs to Item objects
 *
 * Used by validators to look up item properties for existing placements.
 */
using ItemRegistry = std::unordered_map<std::string, Item>;

/**
 * @brief Abstract interface for placement constraint validators
 *
 * Validators check whether a proposed placement satisfies specific constraints.
 */
class IConstraintValidator {
public:
    virtual ~IConstraintValidator() = default;

    /**
     * @brief Check if a placement is valid
     *
     * @param item Item being placed
     * @param proposed Proposed placement
     * @param existing Existing placements in the same bin
     * @param bin Bin being packed into
     * @param registry Optional registry for looking up item properties by ID
     * @return true if the placement is valid
     */
    [[nodiscard]] virtual bool can_place(const Item& item, const Placement& proposed,
                                         std::span<const Placement> existing, const BinType& bin,
                                         const ItemRegistry& registry = {}) const = 0;

    /**
     * @brief Get the name of this validator (for debugging)
     */
    [[nodiscard]] virtual std::string_view name() const = 0;
};

/**
 * @brief Composite validator that combines multiple validators
 *
 * All contained validators must pass for a placement to be valid.
 */
class CompositeValidator : public IConstraintValidator {
public:
    CompositeValidator() = default;

    /**
     * @brief Add a validator to the composite
     */
    void add(std::unique_ptr<IConstraintValidator> validator);

    /**
     * @brief Add a validator using make_unique
     */
    template <typename T, typename... Args>
    void add(Args&&... args) {
        add(std::make_unique<T>(std::forward<Args>(args)...));
    }

    [[nodiscard]] bool can_place(const Item& item, const Placement& proposed,
                                 std::span<const Placement> existing, const BinType& bin,
                                 const ItemRegistry& registry = {}) const override;

    [[nodiscard]] std::string_view name() const override { return "Composite"; }

    /// Get the number of validators
    [[nodiscard]] std::size_t size() const noexcept { return validators_.size(); }

private:
    std::vector<std::unique_ptr<IConstraintValidator>> validators_;
};

/**
 * @brief Create the default composite validator with all standard validators
 */
[[nodiscard]] std::unique_ptr<CompositeValidator> create_default_validator();

}  // namespace bp3d

#endif  // BP3D_CONSTRAINTS_VALIDATOR_HPP
