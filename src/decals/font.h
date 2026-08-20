#pragma once
#include "draw.h"
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
namespace arnis::decals::font
{
enum class FontSize
{
	S12,
	S18,
	S28,
	S44,
	S64
};
struct Glyph
{
	std::uint16_t x{};
	std::uint8_t width{};
	std::int8_t cursor_offset{};
	std::uint8_t advance{};
};
class Font
{
	std::vector<std::uint8_t> atlas_;
	int atlas_width_{};
	int atlas_height_{};
	std::uint8_t line_height_{};
	std::unordered_map<char32_t, Glyph> glyphs_;

public:
	static std::optional<Font> load(
			FontSize size, const std::filesystem::path &root = "assets/decorations/font");
	static const Font &get(FontSize size);
	int line_height() const { return line_height_; }
	bool covers(const std::string &text) const;
	int width(const std::string &text, int scale = 1) const;
	int draw(Canvas &canvas, int x, int y, const std::string &text,
			std::uint8_t foreground, int scale = 1) const;
	void draw_centered(Canvas &canvas, int center_x, int y, const std::string &text,
			std::uint8_t foreground, int scale = 1) const;
};
struct TextLayout
{
	FontSize size{FontSize::S12};
	int scale{1};
	std::vector<std::string> lines;
	int line_height() const;
	int height() const;
	int width() const;
	void draw_centered(
			Canvas &canvas, int center_x, int center_y, std::uint8_t foreground) const;
};
bool supports(const std::string &text);
std::optional<TextLayout> fit_text(const std::string &text, int max_width, int max_height,
		FontSize largest, bool allow_wrap);
}
