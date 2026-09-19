#include "world_utils.h"
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
	return ".";
}
std::filesystem::path get_bedrock_output_directory()
{
	auto p = home() / "Desktop";
	return std::filesystem::exists(p) ? p : home();
}
std::filesystem::path get_luanti_worlds_directory()
{
	auto p = home() / ".minetest" / "worlds";
	if (std::filesystem::exists(p.parent_path()))
		return p;
	return get_bedrock_output_directory() / "Arnis Luanti Worlds";
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
	while (!out.empty() && std::isspace(static_cast<unsigned char>(out.front())))
		out.erase(out.begin());
	while (!out.empty() &&
			(std::isspace(static_cast<unsigned char>(out.back())) || out.back() == '.'))
		out.pop_back();
	if (out.size() > max_bytes) {
		out.resize(max_bytes);
		while (!out.empty() && (static_cast<unsigned char>(out.back()) & 0xC0) == 0x80)
			out.pop_back();
		while (!out.empty() && (std::isspace(static_cast<unsigned char>(out.back())) ||
									   out.back() == '.'))
			out.pop_back();
	}
	return out.empty() ? "Unknown Location" : out;
}
}
