#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace arnis::mapillary::credits
{
inline constexpr const char *unnamed = "an uploader this facade export does not name";

struct ImageCredit
{
	std::string id;
	std::string title;
	std::string username;
	std::string user_id;

	static ImageCredit by_id(const std::string &id);
	bool names_the_uploader() const;
	std::string image_url() const;
	std::string profile_url() const;
	std::string uploader() const;
	std::string line() const;
};

void reset();
void record(const ImageCredit &);
std::vector<ImageCredit> list();
std::size_t count();
} // namespace arnis::mapillary::credits
