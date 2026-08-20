#include "templates.h"
#include "font.h"
#include <algorithm>
#include <unordered_map>
namespace arnis::decals::templates
{
std::optional<std::string> abbreviate(const std::string &name)
{
	static const std::unordered_map<std::string, std::string> words{{"Street", "St"},
			{"Road", "Rd"}, {"Avenue", "Ave"}, {"Boulevard", "Blvd"}, {"Square", "Sq"},
			{"Station", "Stn"}, {"North", "N"}, {"South", "S"}, {"East", "E"},
			{"West", "W"}};
	std::string out = name;
	bool changed = false;
	for (const auto &[from, to] : words)
		if (auto p = out.find(from); p != std::string::npos) {
			out.replace(p, from.size(), to);
			changed = true;
		}
	return changed ? std::optional{out} : std::nullopt;
}
void blank_plate(Canvas &c, std::uint8_t color)
{
	c.fill(color);
	c.stroke_rect(0, 0, c.width, c.height, 3, colors::NEAR_BLACK);
}
void traffic_sign(Canvas &c, TrafficSign sign)
{
	if (sign == TrafficSign::Stop) {
		c.regular_polygon(64, 64, 58, 8, 3.14159265f / 8, colors::BRIGHT_RED);
		c.regular_polygon(64, 64, 49, 8, 3.14159265f / 8, colors::WHITE);
		c.regular_polygon(64, 64, 44, 8, 3.14159265f / 8, colors::BRIGHT_RED);
	} else if (sign == TrafficSign::GiveWay) {
		c.polygon({{64, 118}, {8, 18}, {120, 18}}, colors::BRIGHT_RED);
		c.polygon({{64, 102}, {24, 29}, {104, 29}}, colors::WHITE);
	} else if (sign == TrafficSign::NoEntry) {
		c.disc(64, 64, 58, colors::BRIGHT_RED);
		c.fill_rect(18, 54, 92, 20, colors::WHITE);
	} else {
		c.disc(64, 64, 58, colors::WHITE);
		c.ring(64, 64, 58, 49, colors::BRIGHT_RED);
	}
}
void speed_limit(Canvas &c, std::uint16_t value, bool mph, SpeedStyle style)
{
	if (style == SpeedStyle::Disc) {
		c.disc(64, 64, 58, colors::WHITE);
		c.ring(64, 64, 58, 48, colors::BRIGHT_RED);
	} else {
		c.rounded_rect(8, 4, 112, 120, 8, colors::WHITE);
		c.stroke_rounded_rect(8, 4, 112, 120, 8, 4, colors::BLACK);
	}
	const auto label = std::to_string(value);
	if (auto layout = font::fit_text(label, 92, 66, font::FontSize::S64, false))
		layout->draw_centered(c, 64, style == SpeedStyle::Disc ? 64 : 70, colors::BLACK);
	if (mph && style != SpeedStyle::Disc)
		font::Font::get(font::FontSize::S12)
				.draw_centered(c, 64, 100, "MPH", colors::BLACK);
}
void route_shield(Canvas &c, ShieldStyle style, const std::string &text)
{
	const auto color = style == ShieldStyle::Yellow	 ? colors::YELLOW
					   : style == ShieldStyle::Green ? colors::SIGN_GREEN
					   : style == ShieldStyle::White ? colors::WHITE
													 : colors::BLUE;
	c.rounded_rect(6, 20, 116, 88, 18, color);
	c.stroke_rounded_rect(6, 20, 116, 88, 18, 4, colors::WHITE);
	if (auto layout = font::fit_text(text, 96, 64, font::FontSize::S44, false))
		layout->draw_centered(c, 64, 64,
				style == ShieldStyle::Yellow || style == ShieldStyle::White
						? colors::BLACK
						: colors::WHITE);
}
void text_sign(Canvas &c, TextStyle style, const std::string &text)
{
	const auto bg =
			style.kind == TextStyleKind::Fascia ? colors::NEAR_BLACK
			: style.kind == TextStyleKind::StreetName && style.blade == BladeStyle::Green
					? colors::SIGN_GREEN
			: style.kind == TextStyleKind::StreetName && style.blade == BladeStyle::Blue
					? colors::BLUE
					: colors::WHITE;
	blank_plate(c, bg);
	const bool dark_text = bg == colors::WHITE;
	if (auto layout = font::fit_text(
				text, int(c.width) - 16, int(c.height) - 12, font::FontSize::S64, true))
		layout->draw_centered(
				c, c.width / 2, c.height / 2, dark_text ? colors::BLACK : colors::WHITE);
}
}
