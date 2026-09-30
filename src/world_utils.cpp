#include "world_utils.h"
#include "retrieve_data.h"
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <system_error>
namespace arnis::world_utils
{
static std::filesystem::path home()
{
	if (const char *h = std::getenv("HOME"))
		return h;
	if (const char *h = std::getenv("USERPROFILE"))
		return h;
	return ".";
}
std::filesystem::path get_bedrock_output_directory()
{
	std::filesystem::path p;
	if (const char *desktop = std::getenv("XDG_DESKTOP_DIR"))
		p = desktop;
	else
		p = home() / "Desktop";
	// Rust's directory resolver returns the preferred Desktop path even on a
	// first run; the caller creates the output directory when exporting.
	return p;
}
std::filesystem::path get_luanti_worlds_directory()
{
	std::filesystem::path base;
#if defined(_WIN32)
	if (const char *appdata = std::getenv("APPDATA"))
		base = std::filesystem::path(appdata) / "Minetest";
#elif defined(__APPLE__)
	if (std::getenv("HOME"))
		base = home() / "Library" / "Application Support" / "minetest";
#else
	if (std::getenv("HOME"))
		base = home() / ".minetest";
#endif
	// Match Rust: only use the platform directory when it can be resolved;
	// otherwise keep generated Luanti worlds in a visible desktop fallback.
	if (!base.empty())
		return base / "worlds";
	std::filesystem::path desktop;
	if (const char *xdg = std::getenv("XDG_DESKTOP_DIR"))
		desktop = xdg;
	else if (!home().empty())
		desktop = home() / "Desktop";
	if (desktop.empty())
		desktop = ".";
	return desktop / "Arnis Luanti Worlds";
}
bool replace_file_atomically(
		const std::filesystem::path &path, const std::vector<std::uint8_t> &bytes)
{
	if (path.parent_path().empty())
		return false;
	static std::atomic_uint64_t sequence{0};
	const auto temporary = path.string() + ".tmp" + std::to_string(++sequence);
	std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
	if (!output)
		return false;
	if (!bytes.empty())
		output.write(reinterpret_cast<const char *>(bytes.data()),
				static_cast<std::streamsize>(bytes.size()));
	output.close();
	if (!output) {
		std::error_code ignored;
		std::filesystem::remove(temporary, ignored);
		return false;
	}
	std::error_code rename_error;
	std::filesystem::rename(temporary, path, rename_error);
	if (rename_error) {
		std::error_code ignored;
		std::filesystem::remove(temporary, ignored);
		return false;
	}
	return true;
}
std::string sanitize_for_filename(const std::string &name)
{
	constexpr std::size_t max_bytes = 64;
	static constexpr char invalid[] = "<>:\"/\\|?*";
	std::string out;
	out.reserve(std::min(name.size(), max_bytes));
	for (const unsigned char c : name)
		out.push_back(c < 32 || std::strchr(invalid, c) ? '_' : char(c));
	const auto trim = [](std::string &value) {
		std::size_t first = 0;
		while (first < value.size() &&
				std::isspace(static_cast<unsigned char>(value[first])))
			++first;
		std::size_t last = value.size();
		while (last > first &&
				(std::isspace(static_cast<unsigned char>(value[last - 1])) ||
						value[last - 1] == '.'))
			--last;
		value = value.substr(first, last - first);
	};
	trim(out);
	if (out.size() > max_bytes) {
		out.resize(max_bytes);
		// Do not leave a partial UTF-8 sequence after the byte cap. This mirrors
		// Rust's char-boundary truncation and keeps generated world paths valid.
		while (!out.empty() && (static_cast<unsigned char>(out.back()) & 0xc0) == 0x80)
			out.pop_back();
		if (!out.empty() && static_cast<unsigned char>(out.back()) >= 0xc0) {
			const auto lead = static_cast<unsigned char>(out.back());
			const std::size_t width = (lead & 0xe0) == 0xc0	  ? 2
									  : (lead & 0xf0) == 0xe0 ? 3
									  : (lead & 0xf8) == 0xf0 ? 4
															  : 1;
			if (out.size() - 1 + width > max_bytes)
				out.pop_back();
		}
		trim(out);
	}
	return out.empty() ? "Unknown Location" : out;
}

std::string world_folder_name(const std::string &raw)
{
	// Rust caps custom names by characters (not bytes), then applies the same
	// filesystem-safe filtering used for export filenames.  Preserve UTF-8
	// boundaries while enforcing the 48-character UI limit.
	std::string capped;
	std::size_t chars = 0;
	for (std::size_t i = 0; i < raw.size() && chars < 48;) {
		const auto c = static_cast<unsigned char>(raw[i]);
		std::size_t width = c < 0x80 ? 1 : (c < 0xe0 ? 2 : (c < 0xf0 ? 3 : 4));
		if (i + width > raw.size())
			width = 1;
		capped.append(raw, i, width);
		i += width;
		++chars;
	}
	static constexpr char invalid[] = "<>:\"/\\|?*";
	std::string sanitized;
	sanitized.reserve(capped.size());
	for (const unsigned char c : capped)
		sanitized.push_back(c < 32 || std::strchr(invalid, c) ? '_' : char(c));
	const auto trim = [](std::string &value) {
		std::size_t first = 0;
		while (first < value.size() &&
				std::isspace(static_cast<unsigned char>(value[first])))
			++first;
		std::size_t last = value.size();
		while (last > first &&
				(std::isspace(static_cast<unsigned char>(value[last - 1])) ||
						value[last - 1] == '.'))
			--last;
		value = value.substr(first, last - first);
	};
	trim(sanitized);
	return sanitized;
}

std::string unique_world_folder_name(
		const std::filesystem::path &base_path, const std::string &custom_name)
{
	const auto usable = world_folder_name(custom_name);
	if (!usable.empty()) {
		if (!std::filesystem::exists(base_path / usable))
			return usable;
		for (std::uint32_t n = 2;; ++n) {
			const auto candidate = usable + " (" + std::to_string(n) + ")";
			if (!std::filesystem::exists(base_path / candidate))
				return candidate;
		}
	}
	for (std::uint32_t n = 1;; ++n) {
		const auto candidate = "Arnis World " + std::to_string(n);
		if (std::filesystem::exists(base_path / candidate))
			continue;
		bool location_collision = false;
		std::error_code error;
		for (const auto &entry : std::filesystem::directory_iterator(base_path, error)) {
			if (error)
				break;
			const auto name = entry.path().filename().string();
			if (name.rfind(candidate + ": ", 0) == 0) {
				location_collision = true;
				break;
			}
		}
		if (!location_collision)
			return candidate;
	}
}

std::string get_area_name_for_bedrock(const geographic::LLBBox &bbox)
{
	const double lat = (bbox.min().lat() + bbox.max().lat()) * 0.5;
	const double lon = (bbox.min().lng() + bbox.max().lng()) * 0.5;
	return retrieve_data::fetch_area_name(lat, lon).value_or("Unknown Location");
}

std::pair<std::filesystem::path, std::string> build_bedrock_output(
		const geographic::LLBBox &bbox, const std::filesystem::path &output_dir)
{
	const auto safe_name = sanitize_for_filename(get_area_name_for_bedrock(bbox));
	return {output_dir / ("Arnis " + safe_name + ".mcworld"),
			"Arnis World: " + safe_name};
}
}
