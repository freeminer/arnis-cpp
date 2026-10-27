#include "templates.h"

#include "font.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace arnis::decals::templates
{
namespace
{
using font::Font;
using font::FontSize;
using font::TextLayout;

struct Character
{
	std::string bytes;
	char32_t value{};
};

std::vector<Character> characters(std::string_view text)
{
	std::vector<Character> result;
	for (std::size_t i = 0; i < text.size();) {
		const auto first = static_cast<unsigned char>(text[i]);
		std::size_t length = 1;
		char32_t value = first;
		if ((first & 0xE0) == 0xC0 && i + 1 < text.size()) {
			length = 2;
			value = first & 0x1F;
		} else if ((first & 0xF0) == 0xE0 && i + 2 < text.size()) {
			length = 3;
			value = first & 0x0F;
		} else if ((first & 0xF8) == 0xF0 && i + 3 < text.size()) {
			length = 4;
			value = first & 0x07;
		}
		for (std::size_t j = 1; j < length; ++j)
			value = (value << 6) | (static_cast<unsigned char>(text[i + j]) & 0x3F);
		result.push_back({std::string(text.substr(i, length)), value});
		i += length;
	}
	return result;
}

bool whitespace(char32_t c)
{
	return (c >= 0x09 && c <= 0x0D) || c == 0x20 || c == 0x85 || c == 0xA0 ||
		   c == 0x1680 || (c >= 0x2000 && c <= 0x200A) || c == 0x2028 || c == 0x2029 ||
		   c == 0x202F || c == 0x205F || c == 0x3000;
}

std::string join(const std::vector<Character> &chars, std::size_t begin, std::size_t end)
{
	std::string result;
	for (std::size_t i = begin; i < end; ++i)
		result += chars[i].bytes;
	return result;
}

std::optional<TextLayout> fit_truncated_two_lines(
		const std::string &text, int max_width, int max_height, FontSize largest)
{
	auto chars = characters(text);
	while (!chars.empty() && whitespace(chars.front().value))
		chars.erase(chars.begin());
	while (!chars.empty() && whitespace(chars.back().value))
		chars.pop_back();
	if (chars.size() < 2)
		return std::nullopt;

	constexpr std::array<FontSize, 5> sizes = {
			FontSize::S64, FontSize::S44, FontSize::S28, FontSize::S18, FontSize::S12};
	for (const auto size : sizes) {
		if (static_cast<int>(size) > static_cast<int>(largest))
			continue;
		const auto &font = Font::get(size);
		if (font.line_height() * 2 > max_height)
			continue;

		std::size_t first_end = 0;
		for (std::size_t end = 1; end < chars.size(); ++end) {
			if (font.width(join(chars, 0, end), 1) > max_width)
				break;
			first_end = end;
		}
		if (first_end == 0)
			continue;

		std::string line1;
		std::size_t rest_start = 0;
		std::size_t break_at = 0;
		for (std::size_t i = first_end + 1; i-- > 1;) {
			if (whitespace(chars[i].value) || chars[i - 1].value == U'-' ||
					chars[i - 1].value == U'/') {
				break_at = i;
				break;
			}
		}
		if (break_at) {
			line1 = join(chars, 0, break_at);
			while (!line1.empty()) {
				const auto tail = characters(line1);
				if (tail.empty() || !whitespace(tail.back().value))
					break;
				line1.resize(line1.size() - tail.back().bytes.size());
			}
			rest_start = break_at;
		} else {
			std::size_t word_end = chars.size();
			for (std::size_t i = first_end; i < chars.size(); ++i) {
				if (whitespace(chars[i].value) || chars[i].value == U'-' ||
						chars[i].value == U'/') {
					word_end = i;
					break;
				}
			}
			const auto last_cut =
					std::min(first_end, word_end > 3 ? word_end - 3 : std::size_t{0});
			if (last_cut < 3)
				continue;
			std::size_t cut = 0;
			for (std::size_t i = last_cut + 1; i-- > 3;) {
				if (font.width(join(chars, 0, i) + "-", 1) <= max_width) {
					cut = i;
					break;
				}
			}
			if (!cut)
				continue;
			line1 = join(chars, 0, cut) + "-";
			rest_start = cut;
		}
		while (rest_start < chars.size() && whitespace(chars[rest_start].value))
			++rest_start;
		if (rest_start >= chars.size())
			continue;

		const auto remainder = join(chars, rest_start, chars.size());
		if (font.width(remainder, 1) <= max_width)
			return TextLayout{size, 1, {line1, remainder}};

		std::size_t second_end = rest_start;
		for (std::size_t end = rest_start + 1; end <= chars.size(); ++end) {
			std::size_t trimmed_end = end;
			while (trimmed_end > rest_start && whitespace(chars[trimmed_end - 1].value))
				--trimmed_end;
			const auto candidate = join(chars, rest_start, trimmed_end) + "…";
			if (font.width(candidate, 1) > max_width)
				break;
			second_end = end;
		}
		if (second_end == rest_start)
			continue;
		while (second_end > rest_start && whitespace(chars[second_end - 1].value))
			--second_end;
		return TextLayout{size, 1, {line1, join(chars, rest_start, second_end) + "…"}};
	}
	return std::nullopt;
}

constexpr std::array<std::pair<std::string_view, std::string_view>, 34> abbreviations = {{
		{"straße", "str."},
		{"Straße", "Str."},
		{"strasse", "str."},
		{"Strasse", "Str."},
		{"straat", "str."},
		{"Straat", "Str."},
		{"gasse", "g."},
		{"Gasse", "G."},
		{"platz", "pl."},
		{"Platz", "Pl."},
		{"Boulevard", "Blvd"},
		{"boulevard", "blvd"},
		{"Avenue", "Ave"},
		{"avenue", "ave"},
		{"Avenida", "Av."},
		{"Street", "St"},
		{"street", "st"},
		{"Road", "Rd"},
		{"Drive", "Dr"},
		{"Lane", "Ln"},
		{"Place", "Pl"},
		{"Court", "Ct"},
		{"Square", "Sq"},
		{"Terrace", "Ter"},
		{"Crescent", "Cres"},
		{"Highway", "Hwy"},
		{"Parkway", "Pkwy"},
		{"Gardens", "Gdns"},
		{"Chaussee", "Ch."},
		{"Promenade", "Prom."},
		{"Bulevar", "Bul."},
		{"Bulevardul", "Bd."},
		{"Calle", "C/"},
		{"Carrer", "C/"},
}};

std::uint32_t str_hash(std::string_view text)
{
	std::uint32_t hash = 0x811C9DC5u;
	for (const unsigned char byte : text)
		hash = (hash ^ byte) * 0x01000193u;
	return hash;
}

std::optional<TextLayout> fit_or_abbreviate(const std::string &text, int max_width,
		int max_height, FontSize largest, bool wrap)
{
	if (auto layout = font::fit_text(text, max_width, max_height, largest, wrap))
		return layout;
	const auto shortened = abbreviate(text);
	if (shortened)
		if (auto layout =
						font::fit_text(*shortened, max_width, max_height, largest, wrap))
			return layout;
	if (wrap) {
		if (shortened)
			if (auto layout = fit_truncated_two_lines(
						*shortened, max_width, max_height, largest))
				return layout;
		if (auto layout = fit_truncated_two_lines(text, max_width, max_height, largest))
			return layout;
	}

	// Truncate on UTF-8 character boundaries and retain a visible label even when
	// abbreviations cannot fit the plate.
	std::vector<std::size_t> boundaries{0};
	for (std::size_t i = 0; i < text.size();) {
		const auto lead = static_cast<unsigned char>(text[i]);
		std::size_t length = lead < 0x80			 ? 1
							 : (lead & 0xE0) == 0xC0 ? 2
							 : (lead & 0xF0) == 0xE0 ? 3
							 : (lead & 0xF8) == 0xF0 ? 4
													 : 1;
		i = std::min(text.size(), i + length);
		boundaries.push_back(i);
	}
	for (std::size_t chars = boundaries.size() - 1; chars > 3; chars -= 2) {
		std::string candidate = text.substr(0, boundaries[chars]);
		while (!candidate.empty() && candidate.back() == ' ')
			candidate.pop_back();
		candidate += "…";
		if (auto layout =
						font::fit_text(candidate, max_width, max_height, largest, false))
			return layout;
	}
	return std::nullopt;
}

void arrow_right(Canvas &canvas, int x0, int x1, int center_y, int shaft, int head,
		std::uint8_t color)
{
	canvas.fill_rect(x0, center_y - shaft / 2, x1 - head - x0, shaft, color);
	canvas.polygon({{float(x1 - head), float(center_y - head)},
						   {float(x1), float(center_y) + 0.5f},
						   {float(x1 - head), float(center_y + head + 1)}},
			color);
}

void stick_figure(Canvas &canvas, int center_x, int top, int height, std::uint8_t color)
{
	const int head_radius = height / 9;
	canvas.disc(center_x, top + head_radius, head_radius, color);
	const int body_top = top + head_radius * 2 + 1;
	const int hip = top + height * 55 / 100;
	canvas.line(center_x, body_top, center_x, hip, 4, color);
	canvas.line(center_x, hip, center_x - height / 5, top + height, 4, color);
	canvas.line(center_x, hip, center_x + height / 6, top + height * 78 / 100, 4, color);
	canvas.line(center_x + height / 6, top + height * 78 / 100, center_x + height / 8,
			top + height, 4, color);
	canvas.line(center_x, body_top + 4, center_x - height / 5, top + height * 42 / 100, 4,
			color);
	canvas.line(center_x, body_top + 4, center_x + height / 4, top + height * 36 / 100, 4,
			color);
}

void bicycle(Canvas &canvas, int center_x, int center_y, int radius, std::uint8_t color)
{
	const int ring_radius = radius * 22 / 40;
	const int ring_width = std::max(2, radius / 12);
	canvas.ring(center_x - radius + ring_radius, center_y + radius / 3, ring_radius,
			ring_radius - ring_width, color);
	canvas.ring(center_x + radius - ring_radius, center_y + radius / 3, ring_radius,
			ring_radius - ring_width, color);
	const int line_width = std::max(2, radius / 10);
	const int ax = center_x - radius + ring_radius, ay = center_y + radius / 3;
	const int bx = center_x + radius - ring_radius, by = center_y + radius / 3;
	const int px = center_x - radius / 8, py = center_y + radius / 3;
	const int tx = center_x - radius / 4, ty = center_y - radius / 2;
	const int hx = center_x + radius / 2, hy = center_y - radius / 2;
	canvas.line(ax, ay, tx, ty, line_width, color);
	canvas.line(tx, ty, hx, hy, line_width, color);
	canvas.line(hx, hy, bx, by, line_width, color);
	canvas.line(tx, ty, px, py, line_width, color);
	canvas.line(px, py, hx, hy, line_width, color);
	canvas.line(hx, hy, hx + radius / 8, hy - radius / 6, line_width, color);
	canvas.line(tx - radius / 6, ty - radius / 10, tx + radius / 8, ty - radius / 10,
			line_width, color);
}

void shield_text(Canvas &canvas, const std::string &text, int max_width, int max_height,
		int center_y, std::uint8_t color)
{
	if (auto layout = font::fit_text(text, max_width, max_height, FontSize::S44, false))
		layout->draw_centered(canvas, 64, center_y, color);
}
} // namespace

std::optional<std::string> abbreviate(const std::string &name)
{
	std::string output = name;
	bool changed = false;
	for (const auto &[long_form, short_form] : abbreviations) {
		std::string next;
		std::size_t start = 0;
		while (start <= output.size()) {
			const auto end = output.find(' ', start);
			const auto stop = end == std::string::npos ? output.size() : end;
			const auto word = std::string_view(output).substr(start, stop - start);
			if (word == long_form) {
				next.append(short_form);
				changed = true;
			} else if (word.size() > long_form.size() &&
					   word.substr(word.size() - long_form.size()) == long_form) {
				next.append(word.substr(0, word.size() - long_form.size()));
				next.append(short_form);
				changed = true;
			} else {
				next.append(word);
			}
			if (end == std::string::npos)
				break;
			next.push_back(' ');
			start = end + 1;
		}
		output = std::move(next);
	}
	return changed ? std::optional<std::string>(std::move(output)) : std::nullopt;
}

void blank_plate(Canvas &canvas, std::uint8_t color)
{
	canvas.rounded_rect(6, 6, int(TILE) - 12, int(TILE) - 12, 22, color);
	canvas.stroke_rounded_rect(
			6, 6, int(TILE) - 12, int(TILE) - 12, 22, 3, darker(color));
}

void traffic_sign(Canvas &canvas, TrafficSign sign)
{
	using namespace colors;
	constexpr int center = 64;
	switch (sign) {
	case TrafficSign::Stop:
		canvas.regular_polygon(center, center, 60.0f, 8, 3.14159265f / 8.0f, WHITE);
		canvas.regular_polygon(center, center, 55.0f, 8, 3.14159265f / 8.0f, RED);
		Font::get(FontSize::S28)
				.draw_centered(canvas, center, center - 16, "STOP", WHITE, 1);
		break;
	case TrafficSign::GiveWay:
		canvas.polygon({{6, 18}, {122, 18}, {64, 118}}, RED);
		canvas.polygon({{24, 28}, {104, 28}, {64, 98}}, WHITE);
		break;
	case TrafficSign::NoEntry:
		canvas.disc(center, center, 58, WHITE);
		canvas.disc(center, center, 54, RED);
		canvas.rounded_rect(24, center - 10, 80, 20, 3, WHITE);
		break;
	case TrafficSign::PriorityRoad:
		canvas.regular_polygon(center, center, 60.0f, 4, 0.0f, WHITE);
		canvas.regular_polygon(center, center, 50.0f, 4, 0.0f, YELLOW);
		canvas.regular_polygon(center, center, 44.0f, 4, 0.0f, YELLOW);
		break;
	case TrafficSign::Crossing:
		canvas.rounded_rect(12, 12, 104, 104, 6, BLUE);
		canvas.polygon({{64, 20}, {112, 108}, {16, 108}}, WHITE);
		stick_figure(canvas, 66, 50, 54, BLACK);
		break;
	case TrafficSign::OneWay:
		canvas.rounded_rect(4, 40, 120, 48, 4, BLUE);
		canvas.stroke_rounded_rect(4, 40, 120, 48, 4, 2, WHITE);
		arrow_right(canvas, 18, 112, 64, 12, 20, WHITE);
		break;
	case TrafficSign::NoParking:
		canvas.disc(center, center, 58, RED);
		canvas.disc(center, center, 48, BLUE);
		canvas.line(28, 28, 100, 100, 12, RED);
		break;
	case TrafficSign::DeadEnd:
		canvas.rounded_rect(12, 12, 104, 104, 4, BLUE);
		canvas.stroke_rounded_rect(12, 12, 104, 104, 4, 2, WHITE);
		canvas.fill_rect(56, 40, 16, 68, WHITE);
		canvas.fill_rect(30, 28, 68, 14, RED);
		break;
	case TrafficSign::LevelCrossing:
		for (const float flip : {1.0f, -1.0f}) {
			const float sine = std::sin(0.62f) * flip;
			const float cosine = std::cos(0.62f);
			std::vector<std::pair<float, float>> points;
			for (const auto &[x, y] : std::array<std::pair<float, float>, 4>{
						 {{-56, -8}, {56, -8}, {56, 8}, {-56, 8}}})
				points.emplace_back(
						64 + x * cosine - y * sine, 64 + x * sine + y * cosine);
			canvas.polygon(points, WHITE);
		}
		for (const auto &[x, y] : std::array<std::pair<int, int>, 4>{
					 {{14, 33}, {114, 33}, {14, 95}, {114, 95}}})
			canvas.disc(x, y, 7, RED);
		break;
	case TrafficSign::HighVoltage:
		canvas.polygon({{64, 8}, {122, 116}, {6, 116}}, BLACK);
		canvas.polygon({{64, 20}, {112, 110}, {16, 110}}, YELLOW);
		canvas.polygon(
				{{70, 44}, {52, 76}, {64, 76}, {56, 102}, {80, 66}, {68, 66}, {78, 44}},
				BLACK);
		break;
	case TrafficSign::Bicycle:
		canvas.disc(center, center, 58, WHITE);
		canvas.disc(center, center, 54, BLUE);
		bicycle(canvas, center, center, 40, WHITE);
		break;
	case TrafficSign::Motorway:
		canvas.rounded_rect(12, 12, 104, 104, 4, BLUE);
		canvas.stroke_rounded_rect(12, 12, 104, 104, 4, 2, WHITE);
		canvas.fill_rect(44, 30, 10, 74, WHITE);
		canvas.fill_rect(74, 30, 10, 74, WHITE);
		canvas.polygon({{30, 58}, {98, 58}, {92, 68}, {36, 68}}, WHITE);
		break;
	case TrafficSign::MotorwayEnd:
		traffic_sign(canvas, TrafficSign::Motorway);
		canvas.line(24, 104, 104, 24, 10, RED);
		break;
	}
}

void speed_limit(Canvas &canvas, std::uint16_t value, bool mph, SpeedStyle style)
{
	using namespace colors;
	const auto text = std::to_string(value);
	if (style == SpeedStyle::Disc) {
		canvas.disc(64, 64, 58, RED);
		canvas.disc(64, 64, 46, WHITE);
		const auto size = text.size() > 2 ? FontSize::S28 : FontSize::S44;
		const auto &font = Font::get(size);
		const int y = mph ? 60 : 64;
		font.draw_centered(canvas, 64, y - font.line_height() / 2, text, BLACK, 1);
		if (mph)
			Font::get(FontSize::S12).draw_centered(canvas, 64, 88, "mph", BLACK, 1);
		return;
	}

	canvas.rounded_rect(24, 6, 80, 116, 3, WHITE);
	canvas.stroke_rounded_rect(24, 6, 80, 116, 3, 3, BLACK);
	const auto &small = Font::get(FontSize::S12);
	if (style == SpeedStyle::CaPlate)
		small.draw_centered(canvas, 64, 22, "MAXIMUM", BLACK, 1);
	else {
		small.draw_centered(canvas, 64, 14, "SPEED", BLACK, 1);
		small.draw_centered(canvas, 64, 30, "LIMIT", BLACK, 1);
	}
	const auto size = text.size() > 2 ? FontSize::S28 : FontSize::S44;
	const auto &digits = Font::get(size);
	digits.draw_centered(canvas, 64, 90 - digits.line_height() / 2, text, BLACK, 1);
	if (style == SpeedStyle::CaPlate)
		small.draw_centered(canvas, 64, 108, "km/h", BLACK, 1);
}

void route_shield(Canvas &canvas, ShieldStyle style, const std::string &text)
{
	using namespace colors;
	if (style == ShieldStyle::Blue || style == ShieldStyle::Yellow ||
			style == ShieldStyle::Green) {
		const auto plate = style == ShieldStyle::Blue	  ? BLUE
						   : style == ShieldStyle::Yellow ? YELLOW
														  : SIGN_GREEN;
		const auto foreground = style == ShieldStyle::Yellow ? BLACK : WHITE;
		const int width = 110, height = 64;
		canvas.rounded_rect(64 - width / 2, 64 - height / 2, width, height, 6, plate);
		canvas.stroke_rounded_rect(64 - width / 2, 64 - height / 2, width, height, 6, 3,
				style == ShieldStyle::Yellow ? BLACK : WHITE);
		shield_text(canvas, text, width - 16, height - 12, 64, foreground);
		return;
	}
	if (style == ShieldStyle::Interstate) {
		const std::vector<std::pair<float, float>> shield = {
				{14, 20}, {114, 20}, {110, 60}, {96, 96}, {64, 116}, {32, 96}, {18, 60}};
		canvas.polygon(shield, WHITE);
		std::vector<std::pair<float, float>> inner;
		for (const auto &[x, y] : shield)
			inner.emplace_back(64 + (x - 64) * 0.92f, 66 + (y - 66) * 0.92f);
		canvas.polygon(inner, BLUE);
		canvas.polygon({{19, 24}, {109, 24}, {107, 44}, {21, 44}}, RED);
		shield_text(canvas, text, 76, 44, 76, WHITE);
		return;
	}
	const std::vector<std::pair<float, float>> shield = {
			{24, 16}, {104, 16}, {112, 30}, {104, 92}, {64, 116}, {24, 92}, {16, 30}};
	canvas.polygon(shield, BLACK);
	std::vector<std::pair<float, float>> inner;
	for (const auto &[x, y] : shield)
		inner.emplace_back(64 + (x - 64) * 0.9f, 66 + (y - 66) * 0.9f);
	canvas.polygon(inner, WHITE);
	shield_text(canvas, text, 72, 56, 64, BLACK);
}

void text_sign(Canvas &canvas, TextStyle style, const std::string &text)
{
	using namespace colors;
	const int width = static_cast<int>(canvas.width);
	const int center_x = width / 2;
	switch (style.kind) {
	case TextStyleKind::Fascia: {
		constexpr std::array<std::array<std::uint8_t, 3>, 6> schemes = {{
				{{NEAR_BLACK, WHITE, GRAY}},
				{{DARK_RED, WHITE, GOLD}},
				{{DARK_BLUE, WHITE, LIGHT_GRAY}},
				{{SIGN_GREEN, GOLD, GOLD}},
				{{WHITE, BLACK, DARK_GRAY}},
				{{DARK_GRAY, GOLD, GOLD}},
		}};
		const auto &scheme = schemes[str_hash(text) % schemes.size()];
		canvas.rounded_rect(4, 30, width - 8, 68, 6, scheme[0]);
		canvas.stroke_rounded_rect(4, 30, width - 8, 68, 6, 2, scheme[2]);
		if (auto layout = fit_or_abbreviate(text, width - 24, 56, FontSize::S44, true))
			layout->draw_centered(canvas, center_x, 64, scheme[1]);
		break;
	}
	case TextStyleKind::StreetName: {
		const auto plate = style.blade == BladeStyle::Blue	  ? BLUE
						   : style.blade == BladeStyle::Green ? SIGN_GREEN
															  : WHITE;
		const auto foreground = style.blade == BladeStyle::White ? BLACK : WHITE;
		canvas.rounded_rect(4, 74, width - 8, 44, 4, plate);
		canvas.stroke_rounded_rect(4, 74, width - 8, 44, 4, 3, foreground);
		if (auto layout = fit_or_abbreviate(text, width - 20, 36, FontSize::S28, true))
			layout->draw_centered(canvas, center_x, 96, foreground);
		break;
	}
	case TextStyleKind::HouseNumber: {
		const auto &large = Font::get(FontSize::S28);
		const bool use_large = large.width(text, 1) <= 80;
		const auto &selected = Font::get(use_large ? FontSize::S28 : FontSize::S18);
		TextLayout layout{use_large ? FontSize::S28 : FontSize::S18, 1, {text}};
		const int plate_width =
				std::min(width - 4, std::max(40, selected.width(text, 1) + 22));
		canvas.rounded_rect(center_x - plate_width / 2, 44, plate_width, 40, 3, WHITE);
		canvas.stroke_rounded_rect(
				center_x - plate_width / 2, 44, plate_width, 40, 3, 2, BLACK);
		layout.draw_centered(canvas, center_x, 64, BLACK);
		break;
	}
	case TextStyleKind::StationBoard:
		canvas.rounded_rect(2, 36, width - 4, 56, 4, DARK_BLUE);
		canvas.stroke_rounded_rect(2, 36, width - 4, 56, 4, 3, WHITE);
		if (auto layout = fit_or_abbreviate(text, width - 30, 42, FontSize::S44, false))
			layout->draw_centered(canvas, center_x, 64, WHITE);
		break;
	case TextStyleKind::StopName:
		canvas.rounded_rect(4, 46, width - 8, 36, 3, WHITE);
		canvas.stroke_rounded_rect(4, 46, width - 8, 36, 3, 2, DARK_GRAY);
		if (auto layout = fit_or_abbreviate(text, width - 6, 32, FontSize::S28, true))
			layout->draw_centered(canvas, center_x, 64, BLACK);
		break;
	case TextStyleKind::Plaque:
		canvas.rounded_rect(8, 28, width - 16, 72, 4, LIGHT_GRAY);
		canvas.stroke_rounded_rect(8, 28, width - 16, 72, 4, 3, GRAY);
		if (auto layout = fit_or_abbreviate(text, width - 28, 58, FontSize::S18, true))
			layout->draw_centered(canvas, center_x, 64, NEAR_BLACK);
		break;
	}
}
} // namespace arnis::decals::templates
