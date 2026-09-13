#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace arnis::overture::cache
{

bool valid_release(const std::string &release);
std::optional<std::filesystem::path> release_dir(
		const std::filesystem::path &root, const std::string &release);
std::optional<std::vector<std::uint8_t>> read(const std::filesystem::path &path);
bool write_atomic(
		const std::filesystem::path &path, const std::vector<std::uint8_t> &bytes);
std::optional<std::string> last_good_release(const std::filesystem::path &root);
bool set_last_good_release(const std::filesystem::path &root, const std::string &release);
void prune_releases_older_than(
		const std::filesystem::path &root, const std::string &keep);

} // namespace arnis::overture::cache
