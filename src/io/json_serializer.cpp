/**
 * @file json_serializer.cpp
 * @brief JSON serialization implementation
 */

#include "bp3d/io/json_serializer.hpp"

#include "bp3d/result.hpp"
#include "bp3d/rotation.hpp"
#include "bp3d/types.hpp"
#include "nlohmann/json_fwd.hpp"

#include <bitset>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bp3d {

// NOLINTNEXTLINE(readability-identifier-naming) - 'json' is the idiomatic nlohmann alias.
using json = nlohmann::json;

// ============================================================================
// Item constraints JSON conversion
// ============================================================================

void to_json(json& j, const ItemConstraints& c) {
    j = json{{"this_side_up", c.this_side_up},
             {"stackable", c.stackable},
             {"max_stack_weight", c.max_stack_weight},
             {"allowed_rotations", c.allowed_rotations.to_ulong()}};
}

void from_json(const json& j, ItemConstraints& c) {
    if (j.contains("this_side_up")) {
        j.at("this_side_up").get_to(c.this_side_up);
    }
    if (j.contains("stackable")) {
        j.at("stackable").get_to(c.stackable);
    }
    if (j.contains("max_stack_weight")) {
        j.at("max_stack_weight").get_to(c.max_stack_weight);
    }
    if (j.contains("allowed_rotations")) {
        c.allowed_rotations = std::bitset<6>(j.at("allowed_rotations").get<unsigned long>());
    }
}

// ============================================================================
// Dimensions JSON conversion
// ============================================================================

void to_json(json& j, const Dimensions& d) {
    j = json{{"width", d.width}, {"height", d.height}, {"depth", d.depth}};
}

void from_json(const json& j, Dimensions& d) {
    j.at("width").get_to(d.width);
    j.at("height").get_to(d.height);
    j.at("depth").get_to(d.depth);
}

// ============================================================================
// Vector3 JSON conversion
// ============================================================================

void to_json(json& j, const Vector3& v) {
    j = json{{"x", v.x}, {"y", v.y}, {"z", v.z}};
}

void from_json(const json& j, Vector3& v) {
    j.at("x").get_to(v.x);
    j.at("y").get_to(v.y);
    j.at("z").get_to(v.z);
}

// ============================================================================
// Item JSON conversion
// ============================================================================

void to_json(json& j, const Item& item) {
    j = json{{"id", item.id},
             {"width", item.dimensions.width},
             {"height", item.dimensions.height},
             {"depth", item.dimensions.depth},
             {"weight", item.weight},
             {"quantity", item.quantity},
             {"constraints", item.constraints}};
}

void from_json(const json& j, Item& item) {
    j.at("id").get_to(item.id);

    // Support both nested and flat dimensions
    if (j.contains("dimensions")) {
        j.at("dimensions").get_to(item.dimensions);
    } else {
        j.at("width").get_to(item.dimensions.width);
        j.at("height").get_to(item.dimensions.height);
        j.at("depth").get_to(item.dimensions.depth);
    }

    if (j.contains("weight")) {
        j.at("weight").get_to(item.weight);
    }
    if (j.contains("quantity")) {
        j.at("quantity").get_to(item.quantity);
    }
    if (j.contains("constraints")) {
        j.at("constraints").get_to(item.constraints);
    }
}

// ============================================================================
// BinType JSON conversion
// ============================================================================

void to_json(json& j, const BinType& bin) {
    j = json{{"id", bin.id},
             {"width", bin.dimensions.width},
             {"height", bin.dimensions.height},
             {"depth", bin.dimensions.depth},
             {"max_weight", bin.max_weight},
             {"cost", bin.cost}};
}

void from_json(const json& j, BinType& bin) {
    j.at("id").get_to(bin.id);

    if (j.contains("dimensions")) {
        j.at("dimensions").get_to(bin.dimensions);
    } else {
        j.at("width").get_to(bin.dimensions.width);
        j.at("height").get_to(bin.dimensions.height);
        j.at("depth").get_to(bin.dimensions.depth);
    }

    if (j.contains("max_weight")) {
        j.at("max_weight").get_to(bin.max_weight);
    }
    if (j.contains("cost")) {
        j.at("cost").get_to(bin.cost);
    }
}

// ============================================================================
// Placement JSON conversion
// ============================================================================

void to_json(json& j, const Placement& p) {
    j = json{{"item_id", p.item_id},
             {"bin_id", p.bin_id},
             {"bin_index", p.bin_index},
             {"position", p.position},
             {"rotation", std::string(rotation_name(p.rotation))},
             {"rotated_dimensions", p.rotated_dimensions}};
}

void from_json(const json& j, Placement& p) {
    j.at("item_id").get_to(p.item_id);
    j.at("bin_id").get_to(p.bin_id);
    j.at("bin_index").get_to(p.bin_index);
    j.at("position").get_to(p.position);
    j.at("rotated_dimensions").get_to(p.rotated_dimensions);

    // Parse rotation from string
    std::string const rot_str = j.at("rotation").get<std::string>();
    if (rot_str == "WDH") {
        p.rotation = Rotation::WDH;
    } else if (rot_str == "HWD") {
        p.rotation = Rotation::HWD;
    } else if (rot_str == "HDW") {
        p.rotation = Rotation::HDW;
    } else if (rot_str == "DWH") {
        p.rotation = Rotation::DWH;
    } else if (rot_str == "DHW") {
        p.rotation = Rotation::DHW;
    } else {
        // Default to WHD for "WHD" and any unrecognized value.
        p.rotation = Rotation::WHD;
    }
}

// ============================================================================
// PackingStats JSON conversion
// ============================================================================

void to_json(json& j, const PackingStats& s) {
    j = json{{"utilization", s.utilization},
             {"total_item_volume", s.total_item_volume},
             {"total_bin_volume", s.total_bin_volume},
             {"total_cost", s.total_cost},
             {"execution_time_us", s.execution_time.count()}};
}

// ============================================================================
// PackingResult JSON conversion
// ============================================================================

void to_json(json& j, const PackingResult& r) {
    j = json{{"algorithm", r.algorithm},
             {"bins_used", r.bins_used},
             {"placements", r.placements},
             {"unpacked_items", r.unpacked_items},
             {"stats", r.stats}};
}

// ============================================================================
// Public API functions
// ============================================================================

std::string items_to_json(std::span<const Item> items) {
    json j = json::array();
    for (const auto& item : items) {
        j.push_back(item);
    }
    return j.dump(2);
}

std::string bin_types_to_json(std::span<const BinType> bins) {
    json j = json::array();
    for (const auto& bin : bins) {
        j.push_back(bin);
    }
    return j.dump(2);
}

std::string result_to_json(const PackingResult& result) {
    json const j = result;
    return j.dump(2);
}

std::vector<Item> items_from_json(std::string_view json_str) {
    json const j = json::parse(json_str);
    return j.get<std::vector<Item>>();
}

std::vector<BinType> bin_types_from_json(std::string_view json_str) {
    json const j = json::parse(json_str);
    return j.get<std::vector<BinType>>();
}

PackingInput input_from_json(std::string_view json_str) {
    json j = json::parse(json_str);
    PackingInput input;

    if (j.contains("items")) {
        input.items = j.at("items").get<std::vector<Item>>();
    }
    if (j.contains("bins")) {
        input.bins = j.at("bins").get<std::vector<BinType>>();
    }

    return input;
}

PackingInput load_input_file(const std::filesystem::path& path) {
    std::ifstream const file(path);
    if (!file) {
        throw std::runtime_error("Cannot open file: " + path.string());
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return input_from_json(buffer.str());
}

void save_result_file(const PackingResult& result, const std::filesystem::path& path) {
    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("Cannot create file: " + path.string());
    }

    file << result_to_json(result);
}

}  // namespace bp3d
