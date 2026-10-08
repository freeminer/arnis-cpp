#pragma once

#include "../../../arnis_adapter.h"
#include "../decals/region.h"

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace arnis::road_markings
{
struct TransverseMark
{
	enum class Kind
	{
		Stop,
		GiveWay,
		Zebra,
		CrossingLines
	};
	std::uint32_t t{0};
	Kind kind{Kind::Stop};
	bool broken{false};
	std::int8_t side{0};

	bool covers(std::uint32_t path_index) const
	{
		// Rust stores every transverse row as its own mark; width is not inferred
		// from the kind. Zebra bars and paired edge lines therefore cover only
		// the path indices explicitly emitted by RoadMarkingIndex::build.
		return path_index == t;
	}
};

struct WayMarks
{
	std::vector<std::pair<std::uint32_t, std::uint32_t>> gaps;
	std::vector<TransverseMark> marks;
	std::int64_t phase{0};
	std::int8_t phase_step{0};

	bool in_gap(std::uint32_t t) const;
	std::int64_t dash_phase(std::uint32_t t) const;
};

struct CrossingPaint
{
	enum class Kind
	{
		Zebra,
		Lines
	};
	Kind kind{Kind::Zebra};
	bool broken{false};
};

struct CarriagewayClip
{
	std::vector<std::pair<int, int>> centreline;
	int half_width{0};
	std::vector<Block> surface;

	const std::vector<Block> &palette() const { return surface; }
	bool contains(int x, int z) const;
};

class RoadMarkingIndex
{
public:
	bool drives_on_left{false};
	bool yellow_centre{false};
	bool signal_crossing_lines{false};

	static RoadMarkingIndex build(const std::vector<ProcessedElement> &elements,
			double scale, decals::SignRegion region);
	const WayMarks *way(std::uint64_t id) const;
	std::optional<CrossingPaint> crossing_way_paint(const tags_t &tags) const;
	const std::vector<CarriagewayClip> *crossing_clips(std::uint64_t id) const;
	const CarriagewayClip *carriageway_at(std::uint64_t id, int x, int z) const;

private:
	std::unordered_map<std::uint64_t, WayMarks> ways_;
	std::unordered_map<std::uint64_t, std::vector<CarriagewayClip>> crossings_;
};
} // namespace arnis::road_markings
