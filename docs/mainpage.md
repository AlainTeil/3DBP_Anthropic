@mainpage bp3d - 3D Bin Packing Library

@section intro Introduction

**bp3d** is a modern C++20 library for solving the 3D bin packing problem.
It provides multiple algorithms to efficiently pack rectangular items into
bins, minimizing the number of bins used while respecting physical constraints.

@section features Features

- **Multiple Algorithms**: First Fit Decreasing (FFD), Best Fit Decreasing (BFD),
  Guillotine, Extreme Point, and Shelf-based packing
- **Online and Offline Modes**: Stream items one at a time or pack all at once
- **Physical Constraints**: "This side up", "Do not stack", weight limits, load bearing
- **6 Rotation Modes**: All orthogonal rotations supported with constraint filtering
- **Multi-threaded**: Parallel algorithm comparison with a configurable objective
  (minimize bins, minimize cost, or maximize utilization)
- **Spatial Indexing**: Uniform spatial-hash grid accelerates neighbour
  collision and support queries during placement
- **JSON I/O**: Standard JSON format for input/output
- **OBJ Export**: Visualize packing results in any 3D viewer
- **Comprehensive Testing**: 284 tests covering algorithms, constraints, I/O, and edge cases
- **CI/CD**: GitHub Actions with sanitizers (ASan, UBSan, TSan) and clang-tidy

@section quick_start Quick Start

@subsection library_usage Library Usage

```cpp
#include <bp3d/bp3d.hpp>

int main() {
    // Define items to pack
    std::vector<bp3d::Item> items = {
        bp3d::Item{"box1", {10.0, 20.0, 30.0}},
        bp3d::Item{"box2", {15.0, 15.0, 15.0}},
    };

    // Configure solver
    bp3d::SolverConfig config;
    config.bin_types.push_back(bp3d::BinType{"standard", {100.0, 100.0, 100.0}});

    // Create solver and pack
    auto solver = bp3d::create_solver(bp3d::Algorithm::FirstFitDecreasing);
    auto result = solver->solve(items, config);

    // Check results
    std::cout << "Bins used: " << result.bins_used << std::endl;
    std::cout << "Utilization: " << result.utilization_percent() << "%" << std::endl;
}
```

@subsection online_usage Online Packing

```cpp
#include <bp3d/bp3d.hpp>

// Create online solver for streaming items
auto solver = bp3d::create_online_solver(bp3d::Algorithm::ExtremePoint);
solver->reset(config);

// Pack items as they arrive
for (const auto& item : incoming_items) {
    auto placement = solver->pack_one(item);
    if (placement) {
        std::cout << "Placed " << item.id << " in bin " << placement->bin_id << "\n";
    }
}

auto result = solver->finalize();
```

@subsection cli_usage CLI Usage

The CLI is built as `build/<preset>/app/bp3d-cli` and is not installed on
your `PATH` by default; invoke it via its path (examples assume the `debug`
preset, run from the project root). Sample inputs live in `tests/fixtures/`.

```bash
# Pack items from a JSON file
./build/debug/app/bp3d-cli -i tests/fixtures/simple.json -o result.json -a ffd

# Compare all algorithms
./build/debug/app/bp3d-cli -i tests/fixtures/simple.json --compare

# Export to OBJ for visualization
./build/debug/app/bp3d-cli -i tests/fixtures/simple.json --export-obj packing.obj
```

@section algorithms Algorithms

| Algorithm | Type | Best For |
|-----------|------|----------|
| First Fit Decreasing | Offline | General purpose, fast |
| Best Fit Decreasing | Offline | Better utilization |
| Guillotine | Offline | Cutting stock problems |
| Extreme Point | Online/Offline | High quality placements |
| Shelf | Online/Offline | Uniform height items |

@section input_format Input Format

Input JSON format:

```json
{
    "items": [
        {
            "id": "item1",
            "width": 10.0,
            "height": 20.0,
            "depth": 30.0,
            "weight": 5.0,
            "quantity": 1,
            "constraints": {
                "this_side_up": false,
                "stackable": true,
                "max_stack_weight": 100.0
            }
        }
    ],
    "bins": [
        {
            "id": "container",
            "width": 100.0,
            "height": 100.0,
            "depth": 100.0,
            "max_weight": 1000.0
        }
    ]
}
```

@section building Building

Requirements:
- CMake 3.20+
- C++20 compiler (GCC 10+, Clang 10+)

```bash
# Using presets
cmake --preset debug
cmake --build build/debug
ctest --preset debug

# Or directly
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

@section code_quality Code Quality

These targets require a configured build directory, so run `cmake --preset debug`
first if `build/debug` does not exist yet.

```bash
# Reformat all source files in place (modifies files)
cmake --build build/debug --target format

# Verify formatting without modifying files (fails if any file is unformatted)
cmake --build build/debug --target format-check

# Static analysis with clang-tidy
cmake --preset clang-tidy
cmake --build build/clang-tidy
ctest --preset clang-tidy

# AddressSanitizer + UndefinedBehaviorSanitizer
cmake --preset asan
cmake --build build/asan
ctest --preset asan

# ThreadSanitizer
cmake --preset tsan
cmake --build build/tsan
ctest --preset tsan
```

@section ci Continuous Integration

GitHub Actions workflow runs:
- Build and test with GCC and Clang (Debug + Release)
- Static analysis with clang-tidy
- Memory safety checks with AddressSanitizer + UBSan
- Thread safety checks with ThreadSanitizer
- Code formatting verification

@section license License

MIT License - see LICENSE file for details.
