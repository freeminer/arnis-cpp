#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include "tree_library.h"
#include "tree_pack.h"
#include "schematic.h"
#include <filesystem>
#include <memory>
#include <optional>
namespace arnis::trees
{
struct Species
{
	std::string name;
	std::vector<std::string> w1, w2, w3;
};
struct Community
{
	std::string name;
	Habitat habitat;
	std::vector<Species> species;
	unsigned density = 20;
};
bool is_palm(const std::string &name);
bool subtropical_latitude(double latitude);
Habitat habitat_for_land_cover(std::uint8_t land_cover);
const std::vector<std::string> &width_candidates(const Species &, unsigned width);
const Species *choose_species(
		const Community &, unsigned width, std::uint64_t seed, bool subtropical);
std::string choose_schematic(
		const Community &, unsigned width, std::uint64_t seed, bool subtropical);
std::vector<Community> load_communities(const std::filesystem::path &manifest);
class RegionLibrary
{
	std::vector<Community> communities_;

public:
	static RegionLibrary load(const std::filesystem::path &manifest);
	static RegionLibrary combine(
			const std::filesystem::path &first, const std::filesystem::path &second);
	std::string choose(
			Habitat habitat, unsigned width, std::uint64_t seed, bool subtropical) const;
	bool accepts(std::uint64_t seed) const;
	bool place(world_editor::WorldEditor &, const TreePackSource &, Habitat, unsigned,
			std::uint64_t, bool, int, int, int, unsigned rotation = 0) const;
};

// Full region-pack selector matching the Rust `trees/region.rs` path.  The
// older RegionLibrary remains as a lightweight name-only compatibility API.
struct SlotRequest
{
	std::optional<TreeSize> want_size;
	bool density_decided = false;
};
struct SlotSelection
{
	int x = 0, z = 0;
	std::size_t schematic_index = 0;
	unsigned rotation = 0;
};
class RegionSelector
{
	struct Data;
	std::shared_ptr<Data> data_;
	explicit RegionSelector(std::shared_ptr<Data> data) : data_(std::move(data)) {}

public:
	RegionSelector() = default;
	static std::optional<RegionSelector> load(const TreePackSource &, double scale,
			int ground_level, const SizeFilter &sizes = SizeFilter{},
			bool exclude_palms = false);
	static std::optional<RegionSelector> load_for_location(double latitude,
			double longitude, const std::filesystem::path &root, double scale,
			int ground_level, const SizeFilter &sizes = SizeFilter{});
	bool empty() const;
	std::size_t entry_count() const;
	int base_spacing() const;
	std::optional<SlotSelection> pick_slot(
			int x, int z, Habitat, int elevation_y, SlotRequest request = {}) const;
	const Schematic *schematic(std::size_t index) const;
};
}
