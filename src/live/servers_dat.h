#pragma once

#include <filesystem>
#include <string>

namespace arnis::live::servers_dat
{
inline constexpr const char *ENTRY_NAME = "Arnis Live";

// Put the Arnis Live entry first, or update its address without changing other
// multiplayer entries. Existing servers.dat files are replaced atomically.
bool ensure_entry(const std::filesystem::path &game_dir, const std::string &address,
		std::string *error = nullptr);
} // namespace arnis::live::servers_dat
