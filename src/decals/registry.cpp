#include "registry.h"

#include <algorithm>

namespace arnis::decals
{
DecalKey DecalKey::text(TextStyle style, std::string value, std::uint8_t cols)
{
	return TextKey{style, std::move(value), std::max<std::uint8_t>(1, cols)};
}
std::pair<std::uint32_t, std::uint32_t> DecalKey::dims() const
{
	return std::visit(
			[](const auto &key) -> std::pair<std::uint32_t, std::uint32_t> {
				using Key = std::decay_t<decltype(key)>;
				if constexpr (std::is_same_v<Key, TextKey>)
					return {key.cols, 1};
				if constexpr (std::is_same_v<Key, PosterKey>)
					return {3, 2};
				if constexpr (std::is_same_v<Key, ColumnPosterKey>)
					return {1, 2};
				if constexpr (std::is_same_v<Key, LocalMapKey>)
					return {2, 2};
				return {1, 1};
			},
			*this);
}
int DecalEntry::tile_id(std::uint32_t col, std::uint32_t row) const
{
	return base_id + int(row * cols + col);
}
DecalRegistry DecalRegistry::from_keys(const std::set<DecalKey> &keys)
{
	DecalRegistry registry;
	for (const auto &key : keys) {
		const auto [cols, rows] = key.dims();
		registry.entries_.emplace(key, DecalEntry{registry.next_id_, cols, rows});
		registry.next_id_ += int(cols * rows);
		registry.ordered_.push_back(key);
	}
	return registry;
}
std::optional<DecalEntry> DecalRegistry::get(const DecalKey &key) const
{
	const auto found = entries_.find(key);
	return found == entries_.end() ? std::nullopt : std::optional{found->second};
}
bool DecalRegistry::contains(const DecalKey &key) const
{
	return entries_.contains(key);
}
std::size_t DecalRegistry::size() const
{
	return ordered_.size();
}
bool DecalRegistry::empty() const
{
	return ordered_.empty();
}
int DecalRegistry::max_id() const
{
	return next_id_ - 1;
}
int DecalRegistry::tile_count() const
{
	return next_id_ - FIRST_ID;
}
const std::vector<DecalKey> &DecalRegistry::ordered() const
{
	return ordered_;
}
}
