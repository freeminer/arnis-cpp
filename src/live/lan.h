#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace arnis::live::lan
{
inline constexpr std::uint16_t DEFAULT_PORT = 25565;

std::string address(const std::string &host, std::uint16_t port);

class Lan
{
public:
	using AddressCallback = std::function<void(std::optional<std::string>)>;

	static std::unique_ptr<Lan> bind(
			std::optional<std::uint16_t> preferred, std::string &error);
	~Lan();
	Lan(const Lan &) = delete;
	Lan &operator=(const Lan &) = delete;

	std::uint16_t port() const;
	bool open(std::uint16_t server_port, const std::string &motd,
			AddressCallback on_address, std::string &error);
	bool logged_in_here(const std::string &name) const;

private:
	struct Shared;
	Lan(std::uint16_t port, std::shared_ptr<Shared> shared);

	std::uint16_t port_;
	std::shared_ptr<Shared> shared_;
	std::thread accept_thread_;
	std::thread announce_thread_;
};
} // namespace arnis::live::lan
