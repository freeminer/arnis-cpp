#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace arnis::live::rcon
{
struct PlayerPos
{
	std::string name;
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

class Rcon
{
public:
	static std::unique_ptr<Rcon> connect(
			std::uint16_t port, const std::string &password, std::string &error);
	~Rcon();
	Rcon(const Rcon &) = delete;
	Rcon &operator=(const Rcon &) = delete;
	Rcon(Rcon &&) noexcept;
	Rcon &operator=(Rcon &&) noexcept;

	std::optional<std::string> cmd(const std::string &command, std::string &error);

private:
	struct State;
	explicit Rcon(std::unique_ptr<State> state);
	std::unique_ptr<State> state_;
};

std::vector<PlayerPos> parse_positions(const std::string &reply);
std::vector<std::string> parse_list(const std::string &reply);
std::string text_component(const std::string &message, const std::string &color);
} // namespace arnis::live::rcon
