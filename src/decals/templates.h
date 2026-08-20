#pragma once
#include "draw.h"
#include "registry.h"
#include <optional>
#include <string>
namespace arnis::decals::templates
{
std::optional<std::string> abbreviate(const std::string &name);
void blank_plate(Canvas &canvas, std::uint8_t color);
void traffic_sign(Canvas &canvas, TrafficSign sign);
void speed_limit(Canvas &canvas, std::uint16_t value, bool mph, SpeedStyle style);
void route_shield(Canvas &canvas, ShieldStyle style, const std::string &text);
void text_sign(Canvas &canvas, TextStyle style, const std::string &text);
}
