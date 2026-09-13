#include "credits.h"

#include <map>
#include <mutex>

namespace arnis::mapillary::credits
{
namespace
{
std::map<std::string, ImageCredit> &store()
{
	static std::map<std::string, ImageCredit> value;
	return value;
}
std::mutex &store_mutex()
{
	static std::mutex value;
	return value;
}
} // namespace

ImageCredit ImageCredit::by_id(const std::string &id)
{
	return {id, id, {}, {}};
}
bool ImageCredit::names_the_uploader() const
{
	return !username.empty() || !user_id.empty();
}
std::string ImageCredit::image_url() const
{
	return "https://www.mapillary.com/app/?pKey=" + id + "&focus=photo";
}
std::string ImageCredit::profile_url() const
{
	if (!names_the_uploader())
		return {};
	return "https://www.mapillary.com/app/user/" +
		   (!username.empty() ? username : user_id);
}
std::string ImageCredit::uploader() const
{
	return !username.empty() ? username : (!user_id.empty() ? "unknown" : unnamed);
}
std::string ImageCredit::line() const
{
	std::string result = title + " (" + image_url() + ") by " + uploader();
	if (names_the_uploader())
		result += " (" + profile_url() + ")";
	return result + ", licensed under CC-BY-SA";
}
void reset()
{
	std::lock_guard<std::mutex> lock(store_mutex());
	store().clear();
}
void record(const ImageCredit &credit)
{
	if (credit.id.empty())
		return;
	std::lock_guard<std::mutex> lock(store_mutex());
	store().emplace(credit.id, credit);
}
std::vector<ImageCredit> list()
{
	std::lock_guard<std::mutex> lock(store_mutex());
	std::vector<ImageCredit> result;
	result.reserve(store().size());
	for (const auto &[id, credit] : store())
		result.push_back(credit);
	return result;
}
std::size_t count()
{
	std::lock_guard<std::mutex> lock(store_mutex());
	return store().size();
}
} // namespace arnis::mapillary::credits
