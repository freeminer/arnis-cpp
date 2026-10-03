#include "assets_root.h"

#include <mutex>
#include <utility>

namespace arnis::assets
{
namespace
{
std::mutex base_mutex;
std::filesystem::path configured_base;

std::filesystem::path source_tree_base()
{
	const auto source_assets =
			std::filesystem::path(__FILE__).parent_path().parent_path() / "assets";
	const auto working_arnis_assets =
			std::filesystem::current_path() / "assets" / "arnis";
	if (std::filesystem::is_directory(working_arnis_assets))
		return working_arnis_assets;
	if (std::filesystem::is_directory(source_assets))
		return source_assets;
	const auto working_assets = std::filesystem::current_path() / "assets";
	if (std::filesystem::is_directory(working_assets))
		return working_assets;
	return source_assets;
}
} // namespace

void set_base_directory(std::filesystem::path path)
{
	const std::lock_guard<std::mutex> lock(base_mutex);
	configured_base = std::move(path);
}

std::filesystem::path base_directory()
{
	const std::lock_guard<std::mutex> lock(base_mutex);
	return configured_base.empty() ? source_tree_base() : configured_base;
}

std::filesystem::path path(std::filesystem::path relative_path)
{
	if (relative_path.is_absolute())
		return relative_path;
	if (relative_path.begin() != relative_path.end() &&
			*relative_path.begin() == "assets")
		relative_path = relative_path.lexically_relative("assets");
	const auto base = base_directory();
	const auto direct = base / relative_path;
	std::error_code error;
	if (std::filesystem::exists(direct, error))
		return direct;
	// Development/package staging may pass the Arnis repository root
	// (share/assets/arnis), whose payload is still nested under assets/.
	// Installed flat roots (share/assets/arnis/climate, ... ) take precedence.
	const auto nested = base / "assets" / relative_path;
	error.clear();
	if (std::filesystem::exists(nested, error))
		return nested;
	return direct;
}
} // namespace arnis::assets
