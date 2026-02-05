/**
 * @file solver.hpp
 * @brief Solver interfaces: ISolver, IOnlineSolver
 */

#ifndef BP3D_SOLVER_HPP
#define BP3D_SOLVER_HPP

#include "bp3d/config.hpp"
#include "bp3d/result.hpp"
#include "bp3d/types.hpp"

#include <memory>
#include <optional>
#include <span>
#include <string_view>

namespace bp3d {

/**
 * @brief Abstract interface for bin packing solvers
 *
 * This is the base interface for all packing algorithms.
 * Implementations should be stateless - all state is managed through
 * the parameters and return values.
 */
class ISolver {
public:
    virtual ~ISolver() = default;

    // Non-copyable but movable
    ISolver() = default;
    ISolver(const ISolver&) = delete;
    ISolver& operator=(const ISolver&) = delete;
    ISolver(ISolver&&) = default;
    ISolver& operator=(ISolver&&) = default;

    /**
     * @brief Solve the bin packing problem
     *
     * @param items Items to pack
     * @param config Solver configuration
     * @return Packing result with placements and statistics
     */
    [[nodiscard]] virtual PackingResult solve(std::span<const Item> items,
                                              const SolverConfig& config) = 0;

    /**
     * @brief Get the name of this algorithm
     */
    [[nodiscard]] virtual std::string_view name() const = 0;

    /**
     * @brief Whether this solver supports online (streaming) packing
     */
    [[nodiscard]] virtual bool supports_online() const = 0;
};

/**
 * @brief Interface for online (streaming) bin packing
 *
 * Online solvers can pack items one at a time as they arrive,
 * without knowing the full set of items in advance.
 */
class IOnlineSolver : public ISolver {
public:
    /**
     * @brief Reset the solver state for a new packing session
     *
     * @param config Solver configuration
     */
    virtual void reset(const SolverConfig& config) = 0;

    /**
     * @brief Try to pack a single item
     *
     * @param item Item to pack
     * @return Placement if successful, std::nullopt if item couldn't be placed
     */
    [[nodiscard]] virtual std::optional<Placement> pack_one(const Item& item) = 0;

    /**
     * @brief Finalize packing and get results
     *
     * @return Complete packing result
     */
    [[nodiscard]] virtual PackingResult finalize() = 0;

    // IOnlineSolver always supports online mode
    [[nodiscard]] bool supports_online() const override { return true; }
};

/**
 * @brief Create a solver instance for the specified algorithm
 *
 * @param algorithm Algorithm to use
 * @return Solver instance
 * @throws std::invalid_argument if algorithm is not supported
 */
[[nodiscard]] std::unique_ptr<ISolver> create_solver(Algorithm algorithm);

/**
 * @brief Create an online solver instance for the specified algorithm
 *
 * @param algorithm Algorithm to use (must support online mode)
 * @return Online solver instance
 * @throws std::invalid_argument if algorithm doesn't support online mode
 */
[[nodiscard]] std::unique_ptr<IOnlineSolver> create_online_solver(Algorithm algorithm);

/**
 * @brief Check if an algorithm supports online packing
 */
[[nodiscard]] bool algorithm_supports_online(Algorithm algorithm);

}  // namespace bp3d

#endif  // BP3D_SOLVER_HPP
