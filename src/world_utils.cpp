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
	else
		base = home() / "AppData" / "Roaming" / "Minetest";
#elif defined(__APPLE__)
	base = home() / "Library" / "Application Support" / "minetest";
#else
	base = home() / ".minetest";
#endif
	// Match Rust: a resolvable platform data directory is authoritative even
	// before the worlds directory exists; callers create it during world setup.
	return base / "worlds";
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
