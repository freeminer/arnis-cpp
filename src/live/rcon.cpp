#include "rcon.h"

#include <boost/asio.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string_view>
#include <thread>
#include <nlohmann/json.hpp>

namespace arnis::live::rcon
{
namespace
{
using Clock = std::chrono::steady_clock;
using Tcp = boost::asio::ip::tcp;
constexpr std::int32_t AUTH = 3;
constexpr std::int32_t EXEC = 2;
constexpr std::size_t FRAGMENT = 4096;
constexpr auto COMMAND_TIMEOUT = std::chrono::seconds(30);

void append_i32_le(std::vector<std::uint8_t> &out, std::int32_t value)
{
	const auto v = static_cast<std::uint32_t>(value);
	for (unsigned i = 0; i < 4; ++i)
		out.push_back(static_cast<std::uint8_t>(v >> (i * 8)));
}

std::int32_t read_i32_le(const std::uint8_t *p)
{
	const auto v = static_cast<std::uint32_t>(p[0]) |
				   (static_cast<std::uint32_t>(p[1]) << 8) |
				   (static_cast<std::uint32_t>(p[2]) << 16) |
				   (static_cast<std::uint32_t>(p[3]) << 24);
	return static_cast<std::int32_t>(v);
}

bool valid_name(const std::string &s)
{
	return !s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c) {
		return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
			   (c >= '0' && c <= '9') || c == '_';
	});
}

std::optional<double> parse_number(std::string_view v)
{
	while (!v.empty() && std::isspace(static_cast<unsigned char>(v.front())))
		v.remove_prefix(1);
	while (!v.empty() && std::isspace(static_cast<unsigned char>(v.back())))
		v.remove_suffix(1);
	if (!v.empty() && (v.back() == 'd' || v.back() == 'D'))
		v.remove_suffix(1);
	while (!v.empty() && std::isspace(static_cast<unsigned char>(v.back())))
		v.remove_suffix(1);
	if (v.empty())
		return std::nullopt;
	std::string copy(v);
	char *end = nullptr;
	const double n = std::strtod(copy.c_str(), &end);
	if (end != copy.c_str() + copy.size() || !std::isfinite(n))
		return std::nullopt;
	return n;
}
} // namespace

struct Client::State
{
	boost::asio::io_context io;
	Tcp::socket socket{io};
	std::int32_t next_id = 1;

	bool write_all(const std::uint8_t *data, std::size_t size, Clock::time_point deadline,
			std::string &error)
	{
		boost::system::error_code ec;
		socket.non_blocking(true, ec);
		if (ec) {
			error = ec.message();
			return false;
		}
		std::size_t sent = 0;
		while (sent < size) {
			const auto n =
					socket.write_some(boost::asio::buffer(data + sent, size - sent), ec);
			if (!ec) {
				sent += n;
				continue;
			}
			if (ec != boost::asio::error::would_block &&
					ec != boost::asio::error::try_again) {
				error = "RCON send: " + ec.message();
				return false;
			}
			if (Clock::now() >= deadline) {
				error = "RCON send timed out";
				return false;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
		}
		return true;
	}

	bool read_all(std::uint8_t *data, std::size_t size, Clock::time_point deadline,
			std::string &error)
	{
		boost::system::error_code ec;
		socket.non_blocking(true, ec);
		if (ec) {
			error = ec.message();
			return false;
		}
		std::size_t received = 0;
		while (received < size) {
			const auto n = socket.read_some(
					boost::asio::buffer(data + received, size - received), ec);
			if (!ec) {
				received += n;
				continue;
			}
			if (ec != boost::asio::error::would_block &&
					ec != boost::asio::error::try_again) {
				error = "RCON read: " + ec.message();
				return false;
			}
			if (Clock::now() >= deadline) {
				error = "RCON read timed out";
				return false;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
		}
		return true;
	}

	std::int32_t send(std::int32_t kind, const std::string &body,
			Clock::time_point deadline, std::string &error)
	{
		const auto id = next_id++;
		if (next_id <= 0)
			next_id = 1;
		std::vector<std::uint8_t> packet;
		packet.reserve(body.size() + 14);
		append_i32_le(packet, static_cast<std::int32_t>(body.size() + 10));
		append_i32_le(packet, id);
		append_i32_le(packet, kind);
		packet.insert(packet.end(), body.begin(), body.end());
		packet.push_back(0);
		packet.push_back(0);
		return write_all(packet.data(), packet.size(), deadline, error) ? id : -1;
	}

	bool receive(std::int32_t &id, std::int32_t &kind, std::string &body,
			Clock::time_point deadline, std::string &error)
	{
		std::array<std::uint8_t, 4> length_bytes{};
		if (!read_all(length_bytes.data(), length_bytes.size(), deadline, error))
			return false;
		const auto length = read_i32_le(length_bytes.data());
		if (length < 10 || length > (1 << 20)) {
			error = "RCON packet of invalid length " + std::to_string(length);
			return false;
		}
		std::vector<std::uint8_t> packet(static_cast<std::size_t>(length));
		if (!read_all(packet.data(), packet.size(), deadline, error))
			return false;
		id = read_i32_le(packet.data());
		kind = read_i32_le(packet.data() + 4);
		body.assign(
				reinterpret_cast<const char *>(packet.data() + 8), packet.size() - 10);
		return true;
	}
};

Client::Client(std::unique_ptr<State> state) : state_(std::move(state))
{
}
Client::~Client() = default;
Client::Client(Client &&) noexcept = default;
Client &Client::operator=(Client &&) noexcept = default;

std::unique_ptr<Client> Client::connect(
		std::uint16_t port, const std::string &password, std::string &error)
{
	auto state = std::make_unique<State>();
	boost::system::error_code ec;
	const auto address = boost::asio::ip::make_address("127.0.0.1", ec);
	if (ec) {
		error = "RCON address: " + ec.message();
		return nullptr;
	}
	state->socket.connect(Tcp::endpoint(address, port), ec);
	if (ec) {
		error = "RCON connect: " + ec.message();
		return nullptr;
	}
	state->socket.set_option(Tcp::no_delay(true), ec);
	const auto deadline = Clock::now() + COMMAND_TIMEOUT;
	const auto id = state->send(AUTH, password, deadline, error);
	if (id < 0)
		return nullptr;
	for (;;) {
		std::int32_t rid = 0, kind = 0;
		std::string body;
		if (!state->receive(rid, kind, body, deadline, error))
			return nullptr;
		if (rid == -1) {
			error = "the server refused the RCON password";
			return nullptr;
		}
		if (rid == id && kind == EXEC)
			return std::unique_ptr<Client>(new Client(std::move(state)));
	}
}

std::optional<std::string> Client::command(const std::string &command, std::string &error)
{
	if (!state_) {
		error = "RCON client is not connected";
		return std::nullopt;
	}
	const auto deadline = Clock::now() + COMMAND_TIMEOUT;
	const auto id = state_->send(EXEC, command, deadline, error);
	if (id < 0)
		return std::nullopt;
	std::string result;
	for (;;) {
		std::int32_t rid = 0, kind = 0;
		std::string part;
		if (!state_->receive(rid, kind, part, deadline, error))
			return std::nullopt;
		if (rid != id)
			continue;
		const bool full = part.size() >= FRAGMENT;
		result += part;
		if (!full)
			return result;
		// Vanilla can emit another packet for a long response. Keep reading briefly,
		// but don't hold up the next command when this was the final full fragment.
		std::int32_t more_id = 0, more_kind = 0;
		std::string more;
		if (!state_->receive(more_id, more_kind, more,
					std::min(deadline, Clock::now() + std::chrono::milliseconds(50)),
					error)) {
			error.clear();
			return result;
		}
		if (more_id != id)
			return result;
		result += more;
		if (more.size() < FRAGMENT)
			return result;
	}
}

std::vector<PlayerPos> parse_positions(const std::string &reply)
{
	constexpr std::string_view marker = " has the following entity data: ";
	std::vector<PlayerPos> result;
	std::size_t rest = 0;
	while ((rest = reply.find(marker, rest)) != std::string::npos) {
		std::size_t name_begin = rest;
		while (name_begin > 0) {
			const unsigned char c = static_cast<unsigned char>(reply[name_begin - 1]);
			if (!std::isalnum(c) && c != '_')
				break;
			--name_begin;
		}
		const std::string name = reply.substr(name_begin, rest - name_begin);
		const auto after = rest + marker.size();
		const auto open = reply.find('[', after);
		const auto close =
				open == std::string::npos ? std::string::npos : reply.find(']', open + 1);
		if (open == std::string::npos || close == std::string::npos)
			break;
		const auto comma1 = reply.find(',', open + 1);
		const auto comma2 = comma1 == std::string::npos ? std::string::npos
														: reply.find(',', comma1 + 1);
		if (comma1 != std::string::npos && comma2 != std::string::npos &&
				comma2 < close) {
			auto x = parse_number(
					std::string_view(reply).substr(open + 1, comma1 - open - 1));
			auto y = parse_number(
					std::string_view(reply).substr(comma1 + 1, comma2 - comma1 - 1));
			auto z = parse_number(
					std::string_view(reply).substr(comma2 + 1, close - comma2 - 1));
			if (valid_name(name) && x && y && z)
				result.push_back({name, *x, *y, *z});
		}
		rest = close + 1;
	}
	return result;
}

std::vector<std::string> parse_list(const std::string &reply)
{
	const auto marker = reply.find(": ");
	if (marker == std::string::npos)
		return {};
	std::vector<std::string> result;
	std::size_t begin = marker + 2;
	while (begin <= reply.size()) {
		const auto end = reply.find(',', begin);
		const auto stop = end == std::string::npos ? reply.size() : end;
		std::size_t first = begin, last = stop;
		while (first < last && std::isspace(static_cast<unsigned char>(reply[first])))
			++first;
		while (last > first && std::isspace(static_cast<unsigned char>(reply[last - 1])))
			--last;
		std::string name = reply.substr(first, last - first);
		if (valid_name(name))
			result.push_back(std::move(name));
		if (end == std::string::npos)
			break;
		begin = end + 1;
	}
	return result;
}

std::string text_component(const std::string &message, const std::string &color)
{
	return nlohmann::json{{"text", message}, {"color", color}}.dump();
}
} // namespace arnis::live::rcon
