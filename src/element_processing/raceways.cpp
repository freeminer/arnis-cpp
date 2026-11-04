#include "raceways.h"

#include "../strict_parse.h"
#include "../unicode_text.h"

#include <algorithm>
#include <string_view>

namespace arnis::raceways
{
namespace
{
enum class Discipline
{
	Circuit,
	Kart,
	Dirt,
	Model,
	Unknown
};

Discipline discipline(const Tags &tags)
{
	const auto found = tags.find("sport");
	if (found == tags.end())
		return Discipline::Unknown;
	const auto &sport = found->second;
	for (std::size_t begin = 0; begin <= sport.size();) {
		const auto sep = sport.find(';', begin);
		const auto end = sep == std::string::npos ? sport.size() : sep;
		const std::string_view item(sport.data() + begin, end - begin);
		const auto [a, b] = unicode_text::trim_whitespace(item);
		const auto name = item.substr(a, b - a);
		if (name == "motor" || name == "motorsport" || name == "motor_racing" ||
				name == "motorcycle_racing" || name == "auto_racing" ||
				name == "drag_racing")
			return Discipline::Circuit;
		if (name == "karting" || name == "kart" || name == "go_kart" || name == "go-kart")
			return Discipline::Kart;
		if (name == "motocross" || name == "enduro" || name == "supercross" ||
				name == "speedway" || name == "bmx" || name == "dirt_bike")
			return Discipline::Dirt;
		if (name == "rc_car" || name == "model" || name == "model_car" ||
				name == "modelrace" || name == "rc")
			return Discipline::Model;
		if (sep == std::string::npos)
			break;
		begin = sep + 1;
	}
	return Discipline::Unknown;
}
}

int default_half_width(const Tags &tags)
{
	switch (discipline(tags)) {
	case Discipline::Circuit:
		return 6;
	case Discipline::Kart:
		return 4;
	case Discipline::Dirt:
		return 3;
	case Discipline::Model:
		return 1;
	case Discipline::Unknown:
		return 5;
	}
	return 5;
}

std::span<const Block> unsurfaced_palette(const Tags &tags)
{
	static const Block dirt[]{COARSE_DIRT, DIRT};
	return discipline(tags) == Discipline::Dirt ? std::span<const Block>(dirt)
												: std::span<const Block>{};
}

std::optional<Block> track_area_block(const Tags &tags)
{
	const auto area = tags.find("area"), type = tags.find("type");
	if ((area == tags.end() || area->second != "yes") &&
			(type == tags.end() || type->second != "multipolygon"))
		return std::nullopt;
	switch (discipline(tags)) {
	case Discipline::Circuit:
	case Discipline::Kart:
	case Discipline::Model:
		return GRAY_CONCRETE_POWDER;
	case Discipline::Dirt:
		return COARSE_DIRT;
	case Discipline::Unknown:
		return std::nullopt;
	}
	return std::nullopt;
}

int deck(const Tags &tags)
{
	const auto layer_tag = tags.find("layer"), bridge = tags.find("bridge");
	const int layer = std::max(0,
			layer_tag == tags.end() ? 0
									: strict_parse::i32(layer_tag->second).value_or(0));
	return bridge != tags.end() && bridge->second != "no" ? std::max(1, layer) : layer;
}

bool is_paved(const Tags &tags)
{
	const auto it = tags.find("surface");
	if (it == tags.end())
		return discipline(tags) != Discipline::Dirt;
	const auto &surface = it->second;
	return surface == "asphalt" || surface == "paved" || surface == "concrete" ||
		   surface == "concrete:plates" || surface == "concrete:lanes" ||
		   surface == "cement" || surface == "chipseal" || surface == "bitmac";
}
} // namespace arnis::raceways
