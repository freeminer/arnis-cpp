#pragma once

#include "../bedrock_addon.h"

namespace arnis::bedrock_addon::models
{
constexpr std::uint32_t TEXTURE_WIDTH = 128;
constexpr std::uint32_t TEXTURE_HEIGHT = 64;

bool knows(std::string_view block) noexcept;
std::vector<std::uint8_t> texture();
nlohmann::json geometry(std::string_view name, const std::vector<Shape> &shapes);
} // namespace arnis::bedrock_addon::models
