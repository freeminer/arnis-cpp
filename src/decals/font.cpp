#include "font.h"
#include "../map_item_palette.h"
#include "../../../tinygltf_/stb_image.h"
#include <algorithm>
#include <array>
#include <fstream>
#include <iterator>
#include <stdexcept>
namespace arnis::decals::font
{
namespace
{
const char *size_name(FontSize s)
{
	switch (s) {
	case FontSize::S12:
		return "12";
	case FontSize::S18:
		return "18";
	case FontSize::S28:
		return "28";
	case FontSize::S44:
		return "44";
	case FontSize::S64:
		return "64";
	}
	return "12";
}
std::vector<char32_t> codepoints(const std::string &s)
{
	std::vector<char32_t> out;
	for (std::size_t i = 0; i < s.size();) {
		unsigned char c = s[i++];
		char32_t cp = c;
		if ((c & 0xe0) == 0xc0 && i < s.size())
			cp = ((c & 31) << 6) | (s[i++] & 63);
		else if ((c & 0xf0) == 0xe0 && i + 1 < s.size())
			cp = ((c & 15) << 12) | ((s[i++] & 63) << 6) | (s[i++] & 63);
		else if ((c & 0xf8) == 0xf0 && i + 2 < s.size())
			cp = ((c & 7) << 18) | ((s[i++] & 63) << 12) | ((s[i++] & 63) << 6) |
				 (s[i++] & 63);
		out.push_back(cp);
	}
	return out;
}
std::string trim(std::string s)
{
	const auto a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
	return a == std::string::npos ? std::string{} : s.substr(a, b - a + 1);
}
std::optional<std::pair<std::string, std::string>> split_two(const std::string &s)
{
	std::optional<std::pair<std::size_t, int>> best;
	for (std::size_t i = 1; i + 1 < s.size(); ++i)
		if (s[i] == ' ') {
			int balance = std::abs(int(i) - int(s.size() - 1 - i));
			if (!best || balance < best->second)
				best = {{i, balance}};
		}
	if (!best)
		return std::nullopt;
	return std::pair{s.substr(0, best->first), s.substr(best->first + 1)};
}
}
std::optional<Font> Font::load(FontSize size, const std::filesystem::path &root)
{
	const std::string stem = "dejavu_bold_" + std::string(size_name(size));
	std::ifstream index(root / (stem + ".bin"), std::ios::binary),
			png(root / (stem + ".png"), std::ios::binary);
	if (!index || !png)
		return std::nullopt;
	std::vector<unsigned char> idx((std::istreambuf_iterator<char>(index)), {}),
			bytes((std::istreambuf_iterator<char>(png)), {});
	if (idx.size() < 8 ||
			std::string_view(reinterpret_cast<char *>(idx.data()), 4) != "AFN1")
		return std::nullopt;
	int w = 0, h = 0, channels = 0;
	unsigned char *pixels =
			stbi_load_from_memory(bytes.data(), bytes.size(), &w, &h, &channels, 1);
	if (!pixels)
		return std::nullopt;
	Font font;
	font.atlas_.assign(pixels, pixels + std::size_t(w) * h);
	stbi_image_free(pixels);
	font.atlas_width_ = w;
	font.atlas_height_ = h;
	font.line_height_ = idx[4];
	const std::size_t count = idx[6] | (std::size_t(idx[7]) << 8);
	std::size_t off = 8;
	for (std::size_t n = 0; n < count && off + 9 <= idx.size(); ++n, off += 9) {
		char32_t cp = idx[off] | (char32_t(idx[off + 1]) << 8) |
					  (char32_t(idx[off + 2]) << 16) | (char32_t(idx[off + 3]) << 24);
		font.glyphs_[cp] = {std::uint16_t(idx[off + 4] | (idx[off + 5] << 8)),
				idx[off + 6], std::int8_t(idx[off + 7]), idx[off + 8]};
	}
	return font;
}
const Font &Font::get(FontSize size)
{
	static const std::array<Font, 5> fonts = []() {
		std::array<Font, 5> value;
		for (int i = 0; i < 5; ++i)
			value[i] = Font::load(FontSize(i)).value_or(Font{});
		return value;
	}();
	return fonts[std::size_t(size)];
}
bool Font::covers(const std::string &text) const
{
	for (auto cp : codepoints(text))
		if (!glyphs_.contains(cp))
			return false;
	return true;
}
int Font::width(const std::string &text, int scale) const
{
	int result = 0;
	for (auto cp : codepoints(text))
		if (auto it = glyphs_.find(cp); it != glyphs_.end())
			result += it->second.advance;
	return result * std::max(1, scale);
}
int Font::draw(Canvas &canvas, int x, int y, const std::string &text, std::uint8_t fg,
		int scale) const
{
	scale = std::max(1, scale);
	int cursor = x;
	for (auto cp : codepoints(text)) {
		auto it = glyphs_.find(cp);
		if (it == glyphs_.end())
			continue;
		const auto &g = it->second;
		const int left = cursor - g.cursor_offset * scale;
		for (int gy = 0; gy < line_height_ && gy < atlas_height_; ++gy)
			for (int gx = 0; gx < g.width && g.x + gx < atlas_width_; ++gx) {
				const auto coverage = atlas_[std::size_t(gy) * atlas_width_ + g.x + gx];
				if (coverage < 40)
					continue;
				for (int sy = 0; sy < scale; ++sy)
					for (int sx = 0; sx < scale; ++sx) {
						int px = left + gx * scale + sx, py = y + gy * scale + sy;
						auto under = canvas.get(px, py);
						if (coverage >= 200 || under == TRANSPARENT) {
							if (coverage >= 128)
								canvas.set(px, py, fg);
						} else
							canvas.set(px, py,
									mix(fg, under,
											coverage < 110 ? 1.f / 3.f : 2.f / 3.f));
					}
			}
		cursor += g.advance * scale;
	}
	return cursor - x;
}
void Font::draw_centered(
		Canvas &c, int cx, int y, const std::string &s, std::uint8_t fg, int scale) const
{
	draw(c, cx - width(s, scale) / 2, y, s, fg, scale);
}
int TextLayout::line_height() const
{
	return Font::get(size).line_height() * scale;
}
int TextLayout::height() const
{
	return line_height() * lines.size();
}
int TextLayout::width() const
{
	int w = 0;
	for (const auto &line : lines)
		w = std::max(w, Font::get(size).width(line, scale));
	return w;
}
void TextLayout::draw_centered(Canvas &c, int cx, int cy, std::uint8_t fg) const
{
	int top = cy - height() / 2;
	for (std::size_t i = 0; i < lines.size(); ++i)
		Font::get(size).draw_centered(
				c, cx, top + i * line_height(), lines[i], fg, scale);
}
bool supports(const std::string &text)
{
	return Font::get(FontSize::S18).covers(text);
}
std::optional<TextLayout> fit_text(
		const std::string &input, int max_w, int max_h, FontSize largest, bool wrap)
{
	const auto text = trim(input);
	if (text.empty())
		return std::nullopt;
	const std::array<FontSize, 5> all{
			FontSize::S64, FontSize::S44, FontSize::S28, FontSize::S18, FontSize::S12};
	for (auto size : all) {
		if (int(size) > int(largest))
			continue;
		for (int scale : (size == FontSize::S64 ? std::array<int, 2>{2, 1}
												: std::array<int, 2>{1, 0})) {
			if (!scale)
				continue;
			const auto &font = Font::get(size);
			if (font.line_height() * scale > max_h)
				continue;
			if (font.width(text, scale) <= max_w)
				return TextLayout{size, scale, {text}};
			if (wrap && font.line_height() * scale * 2 <= max_h)
				if (auto lines = split_two(text);
						lines && font.width(lines->first, scale) <= max_w &&
						font.width(lines->second, scale) <= max_w)
					return TextLayout{size, scale, {lines->first, lines->second}};
		}
	}
	return std::nullopt;
}
}
