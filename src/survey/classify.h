#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace arnis::survey::classify
{
inline constexpr std::uint8_t NONE = 0;
inline constexpr std::uint8_t LAWN = 1;
inline constexpr std::uint8_t PAVED = 2;
inline constexpr std::uint8_t HEDGE = 3;
inline constexpr std::uint8_t BUSH = 4;
inline constexpr std::uint8_t TREE = 5;
inline constexpr std::uint8_t SHED = 6;
inline constexpr std::uint8_t POOL = 7;
inline constexpr std::uint8_t BUILDING = 8;
inline constexpr std::uint8_t BED = 9;
inline constexpr std::uint8_t FENCE = 10;
inline constexpr std::uint8_t BARE = 11;
inline constexpr float TUNED_DENSITY = 15.0f;

struct PhotoCell
{
	std::int8_t exg = 0;
	std::uint8_t green = 0, lum = 0;
	std::int8_t blue = 0;
	std::array<std::uint8_t, 3> rgb{};
	bool present = false;
};

struct Inputs
{
	std::size_t w = 0, h = 0;
	std::vector<std::uint8_t> n_all, n_gnd, n_obj, n_bld, n_veg;
	bool laser_plants = false, strict_pools = false, paving_brighter = false;
	std::vector<std::uint8_t> h_obj_q, h_bld_q;
	std::vector<std::uint16_t> i_gnd_q;
	std::vector<PhotoCell> photo;

	Inputs() = default;
	Inputs(std::size_t width, std::size_t height);
	float h_obj(std::size_t c) const;
	float h_bld(std::size_t c) const;
	float i_gnd(std::size_t c) const;
	bool has_photo(std::size_t c) const;
	float exg(std::size_t c) const;
	float green_f(std::size_t c) const;
	float lum(std::size_t c) const;
	float blue(std::size_t c) const;
	const std::array<std::uint8_t, 3> &rgb(std::size_t c) const;
};

struct Out
{
	std::uint8_t cls = 0, height_q = 0;
	std::array<std::uint8_t, 3> color{};
};

struct Counts
{
	std::uint8_t obj = 1, bld = 1, few = 1, flat = 1;
	static Counts at_density(float median_all);
};

enum class Calibration
{
	Photo,
	Laser,
	Defaults
};

float median_density(const Inputs &input);
std::pair<float, float> calibrate(const Inputs &input, std::pair<float, float> defaults,
		Calibration *method = nullptr);
std::optional<float> split_intensity(std::vector<float> &values);
void crowns_not_roofs(Inputs &input);
std::vector<Out> classify(const Inputs &input, std::pair<float, float> intensity_bands);
std::vector<std::array<std::uint8_t, 5>> encode_cells(const std::vector<Out> &cells);

} // namespace arnis::survey::classify
