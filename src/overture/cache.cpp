#include "cache.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <fstream>

namespace arnis::overture::cache
{
namespace
{
constexpr const char *last_good_file = "last_good_release";
std::atomic_uint64_t write_sequence{0};

bool digits(const std::string &value)
{
	return !value.empty() && std::all_of(value.begin(), value.end(), [](unsigned char c) {
		return std::isdigit(c) != 0;
	});
}

std::string sort_key(const std::string &release)
{
	return release;
}
} // namespace

bool valid_release(const std::string &release)
{
	const auto dot = release.find('.');
	if (dot == std::string::npos || release.find('.', dot + 1) != std::string::npos ||
			!digits(release.substr(dot + 1)))
		return false;
	const auto date = release.substr(0, dot);
	if (date.size() != 10 || date[4] != '-' || date[7] != '-')
		return false;
	return digits(date.substr(0, 4)) && digits(date.substr(5, 2)) &&
		   digits(date.substr(8, 2));
}

std::optional<std::filesystem::path> release_dir(
		const std::filesystem::path &root, const std::string &release)
{
	if (!valid_release(release))
		return std::nullopt;
	return root / release;
}

std::optional<std::vector<std::uint8_t>> read(const std::filesystem::path &path)
{
	std::ifstream input(path, std::ios::binary);
	if (!input)
		return std::nullopt;
	return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(input), {});
}

bool write_atomic(
		const std::filesystem::path &path, const std::vector<std::uint8_t> &bytes)
{
	std::error_code error;
	const auto parent = path.parent_path();
	if (parent.empty())
		return false;
	std::filesystem::create_directories(parent, error);
	if (error)
		return false;
	const auto temporary =
			parent / ("." + path.filename().string() + "." +
							 std::to_string(write_sequence.fetch_add(1)) + ".tmp");
	{
		std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
		if (!output)
			return false;
		output.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
		if (!output) {
			output.close();
			std::filesystem::remove(temporary, error);
			return false;
		}
	}
	std::filesystem::rename(temporary, path, error);
	if (!error)
		return true;
	// Windows-style rename cannot replace a destination. This preserves the
	// Rust guarantee of publishing only a fully-written temporary file.
	std::filesystem::remove(path, error);
	error.clear();
	std::filesystem::rename(temporary, path, error);
	if (error)
		std::filesystem::remove(temporary, error);
	return !error;
}

std::optional<std::string> last_good_release(const std::filesystem::path &root)
{
	const auto bytes = read(root / last_good_file);
	if (!bytes)
		return std::nullopt;
	std::string release(bytes->begin(), bytes->end());
	while (!release.empty() && std::isspace(static_cast<unsigned char>(release.back())))
		release.pop_back();
	return valid_release(release) ? std::optional<std::string>(release) : std::nullopt;
}

bool set_last_good_release(const std::filesystem::path &root, const std::string &release)
{
	if (!valid_release(release) || last_good_release(root) == release)
		return false;
	return write_atomic(root / last_good_file,
			std::vector<std::uint8_t>(release.begin(), release.end()));
}

void prune_releases_older_than(const std::filesystem::path &root, const std::string &keep)
{
	if (!valid_release(keep))
		return;
	std::error_code error;
	for (const auto &entry : std::filesystem::directory_iterator(root, error)) {
		if (error)
			return;
		const auto name = entry.path().filename().string();
		if (entry.is_directory(error) && valid_release(name) &&
				sort_key(name) < sort_key(keep))
			std::filesystem::remove_all(entry.path(), error);
	}
}

} // namespace arnis::overture::cache
