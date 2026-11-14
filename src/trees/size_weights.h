#pragma once

#include "tree_library.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cctype>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace arnis::trees
{
// Relative popularity of the schematic size bands. 100 preserves the Rust
// defaults; zero disables a tier even when canopy data requests that size.
struct SizeWeights
{
	std::array<std::uint16_t, 5> percent{100, 100, 100, 100, 100};

	static std::optional<SizeWeights> parse(
			std::string_view spec, std::string *error = nullptr)
	{
		static constexpr std::array<std::string_view, 5> names{
				"small", "medium", "big", "tall", "giant"};
		SizeWeights result;
		std::size_t start = 0;
		while (start <= spec.size()) {
			const auto comma = spec.find(',', start);
			auto token = spec.substr(start, comma == std::string_view::npos
													? spec.size() - start
													: comma - start);
			while (!token.empty() &&
					std::isspace(static_cast<unsigned char>(token.front())))
				token.remove_prefix(1);
			while (!token.empty() &&
					std::isspace(static_cast<unsigned char>(token.back())))
				token.remove_suffix(1);
			if (!token.empty()) {
				const auto equal = token.find('=');
				if (equal == std::string_view::npos) {
					if (error)
						*error = std::string(token) + ": expected name=percent";
					return std::nullopt;
				}
				auto name = token.substr(0, equal);
				auto value = token.substr(equal + 1);
				while (!name.empty() &&
						std::isspace(static_cast<unsigned char>(name.front())))
					name.remove_prefix(1);
				while (!name.empty() &&
						std::isspace(static_cast<unsigned char>(name.back())))
					name.remove_suffix(1);
				std::string normalized(name);
				for (auto &c : normalized)
					c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
				while (!value.empty() &&
						std::isspace(static_cast<unsigned char>(value.front())))
					value.remove_prefix(1);
				while (!value.empty() &&
						std::isspace(static_cast<unsigned char>(value.back())))
					value.remove_suffix(1);
				std::size_t tier = names.size();
				for (std::size_t i = 0; i < names.size(); ++i)
					if (normalized == names[i]) {
						tier = i;
						break;
					}
				unsigned parsed = 0;
				const auto [end, ec] = std::from_chars(
						value.data(), value.data() + value.size(), parsed);
				if (tier == names.size()) {
					if (error)
						*error = normalized +
								 ": expected one of small, medium, big, tall, giant";
					return std::nullopt;
				}
				if (ec != std::errc{} || end != value.data() + value.size() ||
						parsed > 200) {
					if (error)
						*error = std::string(token) + ": percent must be 0-200";
					return std::nullopt;
				}
				result.percent[tier] = static_cast<std::uint16_t>(parsed);
			}
			if (comma == std::string_view::npos)
				break;
			start = comma + 1;
		}
		if (std::all_of(result.percent.begin(), result.percent.end(),
					[](auto p) { return p == 0; })) {
			if (error)
				*error = std::string(spec) + ": give at least one non-zero percent";
			return std::nullopt;
		}
		return result;
	}

	void restrict(SizeFilter &sizes) const
	{
		sizes.small &= percent[0] != 0;
		sizes.medium &= percent[1] != 0;
		sizes.big &= percent[2] != 0;
		sizes.tall &= percent[3] != 0;
		sizes.giant &= percent[4] != 0;
	}

	std::size_t band(std::uint64_t roll) const
	{
		static constexpr std::array<double, 5> shares{150, 350, 330, 135, 35};
		std::array<double, 5> weighted{};
		double sum = 0.0;
		for (std::size_t i = 0; i < weighted.size(); ++i) {
			weighted[i] = shares[i] * percent[i] / 100.0;
			sum += weighted[i];
		}
		const double target = static_cast<double>(roll % 1000) * sum / 1000.0;
		double cumulative = 0.0;
		for (std::size_t i = 0; i < weighted.size(); ++i) {
			cumulative += weighted[i];
			if (target < cumulative)
				return i;
		}
		for (std::size_t i = weighted.size(); i-- > 0;)
			if (weighted[i] > 0.0)
				return i;
		return 0;
	}

	std::string key() const
	{
		std::string out;
		for (std::size_t i = 0; i < percent.size(); ++i) {
			if (i)
				out += ',';
			out += std::to_string(percent[i]);
		}
		return out;
	}
	bool operator==(const SizeWeights &) const = default;
};
}
