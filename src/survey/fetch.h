#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

namespace arnis::survey::fetch
{
inline constexpr const char *PAST_LIMIT = "past the download limit";

struct Failure
{
	enum class Kind
	{
		Missing,
		Unusable,
		Other
	};

	Kind kind = Kind::Other;
	std::string message;
	std::string to_string() const;
	Failure of_part() const;
};

template <typename T>
using Result = std::variant<T, Failure>;

std::uint64_t laser_bytes();
void count_laser_bytes(std::uint64_t bytes);
std::uint64_t photo_bytes();
std::uint64_t remaining_laser_bytes();
class PhotoRequestScope
{
	bool was_photo_thread_;

public:
	PhotoRequestScope();
	~PhotoRequestScope();
	PhotoRequestScope(const PhotoRequestScope &) = delete;
	PhotoRequestScope &operator=(const PhotoRequestScope &) = delete;
};
void limit_laser_bytes(std::uint64_t bytes);
bool past_laser_limit();
Result<bool> within_laser_limit();
std::uint64_t laser_area_m2();
void count_laser_area(double area_m2);
bool unreachable(const std::string &kind);

Result<std::vector<std::uint8_t>> get_range(
		const std::string &url, std::uint64_t start, std::uint64_t length);
Result<std::vector<std::uint8_t>> get_tail(const std::string &url, std::uint64_t length);
Result<std::vector<std::uint8_t>> get_bytes(const std::string &url);
Result<std::uint64_t> download_file(
		const std::string &url, const std::filesystem::path &destination);
std::optional<std::uint64_t> content_length(const std::string &url);
Result<nlohmann::json> get_json(const std::string &url);
} // namespace arnis::survey::fetch
