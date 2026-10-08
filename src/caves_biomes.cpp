#include "caves_biomes.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cctype>
#include <iostream>
#include <limits>

namespace arnis::caves
{
BiomeAmounts BiomeAmounts::parse(std::string_view spec, std::string *error)
{
	BiomeAmounts out;
	if (error)
		error->clear();
	auto fail = [&](std::string s) {
		if (error)
			*error = std::move(s);
	};
	auto trim = [](std::string_view value) {
		const auto first = value.find_first_not_of(" \t\r\n\f\v");
		if (first == std::string_view::npos)
			return std::string_view{};
		const auto last = value.find_last_not_of(" \t\r\n\f\v");
		return value.substr(first, last - first + 1);
	};
	size_t begin = 0;
	while (begin < spec.size()) {
		size_t end = spec.find(',', begin);
		if (end == std::string_view::npos)
			end = spec.size();
		auto part = trim(spec.substr(begin, end - begin));
		if (part.empty()) {
			begin = end + 1;
			continue;
		}
		const auto eq = part.find('=');
		if (eq == std::string_view::npos) {
			fail("'" + std::string(part) + "': expected name=percent");
			return {};
		}
		std::string name(trim(part.substr(0, eq)));
		std::string value(trim(part.substr(eq + 1)));
		try {
			size_t used = 0;
			const double pct = std::stod(value, &used);
			if (used != value.size()) {
				fail("'" + std::string(part) + "': percent is not a number");
				return {};
			}
			if (!std::isfinite(pct)) {
				fail("'" + std::string(part) +
						"': percent must be a finite number (0..=200), got '" + value +
						"'");
				return {};
			}
			const double v = std::clamp(pct, 0.0, 200.0) / 100.0;
			std::transform(name.begin(), name.end(), name.begin(),
					[](unsigned char c) { return char(std::tolower(c)); });
			if (name == "lush")
				out.lush = v;
			else if (name == "dripstone" || name == "drip")
				out.dripstone = v;
			else if (name == "deepdark" || name == "deep_dark" || name == "sculk")
				out.deepdark = v;
			else if (name == "mushroom" || name == "shroom")
				out.mushroom = v;
			else if (name == "ice")
				out.ice = v;
			else if (name == "amethyst" || name == "crystal")
				out.amethyst = v;
			else if (name == "volcanic" || name == "volcano")
				out.volcanic = v;
			else if (name == "coral")
				out.coral = v;
			else {
				fail("unknown cave biome '" + name + "'");
				return {};
			}
		} catch (...) {
			fail("'" + std::string(part) + "': percent is not a number");
			return {};
		}
		begin = end + 1;
	}
	return out;
}
double BiomeAmounts::effective_threshold(double base, double amount)
{
	if (amount <= 0.0 || !std::isfinite(amount))
		return std::numeric_limits<double>::quiet_NaN();
	return std::max(0.02, base - 0.10 * std::log2(amount));
}

BiomeAmounts biome_amounts_for_generation(std::string_view spec)
{
	std::string error;
	const auto amounts = BiomeAmounts::parse(spec, &error);
	if (!error.empty())
		std::clog << "Warning: --cave-biomes ignored (" << error << "); using defaults\n";
	return amounts;
}
}
