# bp3d - 3D Bin Packing Library

A high-performance C++20 library for solving 3D bin packing problems. Implements multiple algorithms supporting both online and offline packing with extensive constraint handling.

## Features

- **Multiple Algorithms**:
  - First Fit Decreasing (FFD)
  - Best Fit Decreasing (BFD)
  - Guillotine-based algorithms
  - Extreme Point methods
  - Shelf algorithms

- **Online & Offline Modes**:
  - Offline: Process all items at once with global optimization
  - Online: Streaming API for real-time packing decisions

- **Constraints Support**:
  - "This side up" - Orientation restrictions
  - "Do not stack" - Prevent items on top
  - Weight/load bearing limits
  - All 6 orthogonal rotations by default

- **Multiple Bin Types**: Support for different container sizes

- **I/O Formats**:
  - JSON input/output
  - Wavefront OBJ export for 3D visualization

- **Performance**:
  - Multi-threaded algorithm execution
  - Optimized for minimizing bin count

- **Test Coverage**: 238 tests covering algorithms, constraints, I/O, and edge cases

## Requirements

- CMake 3.20+
- C++20 compliant compiler (GCC 10+ or Clang 10+)
- Optional: Doxygen (for documentation)

## Building

```bash
# Configure with CMake preset
cmake --preset debug

# Build
cmake --build build/debug

# Run tests
ctest --preset debug

# Or build directly
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
```

## Usage

### Command Line

```bash
# Pack items from JSON file
bp3d-cli --input items.json --output result.json --algorithm ffd

# Use specific bin type
bp3d-cli --input items.json --bins bins.json --algorithm extreme-point

# Compare all algorithms
bp3d-cli --input items.json --compare

# Export visualization
bp3d-cli --input items.json --output result.json --export-obj packing.obj
```

### Library API

```cpp
#include <bp3d/bp3d.hpp>

using namespace bp3d;

// Define items to pack
std::vector<Item> items = {
    {"box1", {10.0, 5.0, 8.0}, 2.5},  // id, dimensions (WxHxD), weight
    {"box2", {6.0, 6.0, 6.0}, 1.0},
    {"box3", {4.0, 4.0, 12.0}, 0.5, 1, {.this_side_up = true}},
};

// Define bin types
std::vector<BinType> bins = {
    {"standard", {100.0, 100.0, 100.0}, 50.0},  // id, dimensions, max_weight
};

// Configure solver
SolverConfig config{
    .bin_types = bins,
    .thread_count = 4,
};

// Create solver and pack
auto solver = create_solver(Algorithm::FirstFitDecreasing);
PackingResult result = solver->solve(items, config);

// Check results
std::cout << "Bins used: " << result.bins_used << "\n";
std::cout << "Utilization: " << result.utilization_percent() << "%\n";
```

### Online Packing

```cpp
#include <bp3d/bp3d.hpp>

using namespace bp3d;

auto solver = create_online_solver(Algorithm::ExtremePoint);
solver->reset(config);

// Pack items as they arrive
for (const auto& item : incoming_items) {
    auto placement = solver->pack_one(item);
    if (placement) {
        std::cout << "Placed " << item.id << " in bin " << placement->bin_id << "\n";
    } else {
        std::cout << "Could not place " << item.id << "\n";
    }
}

auto result = solver->finalize();
```

## Input Format (JSON)

```json
{
    "items": [
        {
            "id": "item1",
            "width": 10.0,
            "height": 5.0,
            "depth": 8.0,
            "weight": 2.5,
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
            "max_weight": 500.0
        }
    ]
}
```

## Algorithms

| Algorithm | Type | Best For |
|-----------|------|----------|
| FFD | Offline | General purpose, good balance |
| BFD | Offline | Better space utilization |
| Guillotine | Offline | Rectangular cutting problems |
| Extreme Point | Online | Real-time packing |
| Shelf | Online | Simple, fast packing |

## Building Documentation

```bash
cmake --preset debug -DBP3D_BUILD_DOCS=ON
cmake --build build/debug --target docs
# Open build/debug/docs/html/index.html
```

## Code Quality

```bash
# Format code
cmake --build build/debug --target format

# Check formatting
cmake --build build/debug --target format-check

# Run with clang-tidy
cmake --preset clang-tidy
cmake --build build/clang-tidy

# Build with AddressSanitizer + UndefinedBehaviorSanitizer
cmake --preset asan
cmake --build build/asan
ctest --preset asan

# Build with ThreadSanitizer
cmake --preset tsan
cmake --build build/tsan
ctest --preset tsan
```

## Continuous Integration

The project includes a GitHub Actions workflow ([.github/workflows/ci.yml](.github/workflows/ci.yml)) that runs:
- Build and test with GCC and Clang (Debug + Release)
- Static analysis with clang-tidy
- Memory safety checks with AddressSanitizer + UBSan
- Thread safety checks with ThreadSanitizer
- Code formatting verification

## License

MIT License
