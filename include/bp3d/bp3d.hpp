/**
 * @file bp3d.hpp
 * @brief Main include file for bp3d library
 *
 * This umbrella header includes all public API headers.
 */

#ifndef BP3D_BP3D_HPP
#define BP3D_BP3D_HPP

// Core types
#include "bp3d/config.hpp"
#include "bp3d/result.hpp"
#include "bp3d/rotation.hpp"
#include "bp3d/types.hpp"

// Solver interfaces
#include "bp3d/solver.hpp"

// Logging
#include "bp3d/logger.hpp"

// I/O
#include "bp3d/io/json_serializer.hpp"
#include "bp3d/io/obj_exporter.hpp"

/**
 * @namespace bp3d
 * @brief 3D bin packing library
 *
 * The bp3d library provides high-performance algorithms for solving
 * 3D bin packing problems with support for:
 * - Multiple packing algorithms (FFD, BFD, Guillotine, Extreme Point, Shelf)
 * - Both online and offline packing modes
 * - Box constraints (orientation, stacking, weight limits)
 * - Multiple bin types
 * - All 6 orthogonal rotations
 * - Multi-threaded execution
 *
 * @par Basic usage:
 * @code
 * #include <bp3d/bp3d.hpp>
 *
 * using namespace bp3d;
 *
 * std::vector<Item> items = {
 *     {"box1", {10.0, 5.0, 8.0}, 2.5},
 *     {"box2", {6.0, 6.0, 6.0}, 1.0},
 * };
 *
 * std::vector<BinType> bins = {
 *     {"standard", {100.0, 100.0, 100.0}, 50.0},
 * };
 *
 * SolverConfig config{.bin_types = bins};
 *
 * auto solver = create_solver(Algorithm::FirstFitDecreasing);
 * PackingResult result = solver->solve(items, config);
 *
 * std::cout << "Bins used: " << result.bins_used << "\n";
 * @endcode
 */

#endif  // BP3D_BP3D_HPP
