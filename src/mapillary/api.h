#pragma once
#include "cache.h"
#include <functional>
#include <map>
namespace arnis::mapillary
{
using JsonFetcher = std::function<std::optional<std::vector<std::uint8_t>>(
		const std::string &, std::size_t)>;

inline constexpr double mapillary_max_cell_deg = .008;
inline constexpr std::size_t mapillary_max_cells = 4096;
struct SearchCell
{
	double min_longitude{}, min_latitude{}, max_longitude{}, max_latitude{};
};
// Graph API bbox searches are deliberately tiled below its 0.01-degree limit.
std::optional<std::vector<SearchCell>> search_cells(
		const SearchCell &bounds, std::size_t maximum = mapillary_max_cells);
std::vector<cache::ImageRecord> parse_search_response(const std::vector<std::uint8_t> &);
std::optional<cache::ImageRecord> parse_image_record(const std::vector<std::uint8_t> &);
class Client
{
	cache::Layout cache_;
	JsonFetcher fetch_;

public:
	Client(cache::Layout cache, JsonFetcher fetch) :
			cache_(std::move(cache)), fetch_(std::move(fetch))
	{
	}
	std::optional<cache::ImageRecord> image(
			const std::string &id, const std::string &url) const;
	std::vector<cache::ImageRecord> search(const SearchCell &,
			const std::string &endpoint, const std::string &token) const;
};
}
