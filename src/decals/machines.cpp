#include "machines.h"

#include "../assets_root.h"

#include <algorithm>

namespace arnis::decals::machines
{
const std::vector<std::string> &names()
{
	static const std::vector<std::string> value{"atm", "coin_press", "dispenser",
			"dog_bags", "ticket_machine", "vending_drinks", "vending_snacks"};
	return value;
}

bool is_small(std::string_view name)
{
	return name == "dispenser" || name == "dog_bags";
}

std::pair<unsigned, unsigned> tiles(std::string_view name)
{
	return {1, is_small(name) ? 1U : 2U};
}

std::optional<std::filesystem::path> asset(
		const std::string &name, const std::filesystem::path &root)
{
	if (std::find(names().begin(), names().end(), name) == names().end())
		return std::nullopt;
	const auto directory = root.empty() ? assets::path("decorations/machines") : root;
	auto path = directory / (name + ".png");
	return std::filesystem::exists(path) ? std::optional{path} : std::nullopt;
}
} // namespace arnis::decals::machines
