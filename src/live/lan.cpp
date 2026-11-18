#include "lan.h"

#include <boost/asio.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arnis::live::lan
{
namespace
{
using Clock = std::chrono::steady_clock;
using Tcp = boost::asio::ip::tcp;
using Udp = boost::asio::ip::udp;
using namespace std::chrono_literals;

constexpr auto ANNOUNCE_EVERY = 1500ms;
constexpr auto HELLO_TIMEOUT = 10s;
constexpr auto IDLE_TIMEOUT = 60s;
constexpr auto ACCEPT_PAUSE = 50ms;
constexpr std::size_t MAX_GUESTS = 64;
constexpr std::size_t MAX_PER_GUEST = 8;
constexpr std::size_t SNIFF_LIMIT = 1024;
const Udp::endpoint GROUP(boost::asio::ip::make_address("224.0.2.60"), 4445);

enum class SniffKind
{
	More,
	Name,
	Done
};
struct Sniff
{
	SniffKind kind = SniffKind::Done;
	std::string name;
};

enum class VarintKind
{
	More,
	Invalid,
	Value
};
struct Varint
{
	VarintKind kind = VarintKind::More;
	std::uint32_t value = 0;
};

Varint read_varint(const std::vector<std::uint8_t> &bytes, std::size_t &at)
{
	std::uint32_t value = 0;
	for (unsigned i = 0; i < 5; ++i) {
		if (at >= bytes.size())
			return {VarintKind::More, 0};
		const auto b = bytes[at++];
		value |= static_cast<std::uint32_t>(b & 0x7f) << (7 * i);
		if ((b & 0x80) == 0)
			return {VarintKind::Value, value};
	}
	return {VarintKind::Invalid, 0};
}

bool valid_utf8(const std::uint8_t *s, std::size_t n)
{
	for (std::size_t i = 0; i < n;) {
		const auto c = s[i++];
		if (c <= 0x7f)
			continue;
		std::uint32_t cp;
		unsigned continuation;
		if (c >= 0xc2 && c <= 0xdf) {
			cp = c & 0x1f;
			continuation = 1;
		} else if (c >= 0xe0 && c <= 0xef) {
			cp = c & 0x0f;
			continuation = 2;
		} else if (c >= 0xf0 && c <= 0xf4) {
			cp = c & 0x07;
			continuation = 3;
		} else {
			return false;
		}
		if (continuation > n - i)
			return false;
		for (unsigned j = 0; j < continuation; ++j) {
			const auto d = s[i++];
			if ((d & 0xc0) != 0x80)
				return false;
			cp = (cp << 6) | (d & 0x3f);
		}
		if ((continuation == 1 && cp < 0x80) || (continuation == 2 && cp < 0x800) ||
				(continuation == 3 && cp < 0x10000) || cp > 0x10ffff ||
				(cp >= 0xd800 && cp <= 0xdfff))
			return false;
	}
	return true;
}

bool read_string(const std::vector<std::uint8_t> &packet, std::size_t &at,
		std::size_t max_chars, std::string &out)
{
	const auto length = read_varint(packet, at);
	if (length.kind != VarintKind::Value || length.value > max_chars * 4 ||
			length.value > packet.size() - std::min(at, packet.size()))
		return false;
	if (length.value == 0) {
		out.clear();
		return true;
	}
	if (!valid_utf8(packet.data() + at, length.value))
		return false;
	out.assign(reinterpret_cast<const char *>(packet.data() + at), length.value);
	at += length.value;
	return true;
}

Sniff login_name(const std::vector<std::uint8_t> &bytes)
{
	std::size_t at = 0;
	const auto frame = [&bytes](std::size_t &offset,
							   std::vector<std::uint8_t> &body) -> SniffKind {
		const auto length = read_varint(bytes, offset);
		if (length.kind == VarintKind::More)
			return SniffKind::More;
		if (length.kind != VarintKind::Value || length.value == 0 ||
				length.value > SNIFF_LIMIT)
			return SniffKind::Done;
		if (length.value > bytes.size() - std::min(offset, bytes.size()))
			return SniffKind::More;
		body.assign(bytes.begin() + offset, bytes.begin() + offset + length.value);
		offset += length.value;
		return SniffKind::Name;
	};

	std::vector<std::uint8_t> handshake;
	auto state = frame(at, handshake);
	if (state != SniffKind::Name)
		return {state, {}};
	std::size_t i = 0;
	auto packet_id = read_varint(handshake, i);
	if (packet_id.kind != VarintKind::Value || packet_id.value != 0)
		return {SniffKind::Done, {}};
	auto protocol = read_varint(handshake, i);
	if (protocol.kind != VarintKind::Value)
		return {SniffKind::Done, {}};
	std::string host;
	if (!read_string(handshake, i, 255, host) || handshake.size() - i < 2)
		return {SniffKind::Done, {}};
	i += 2;
	const auto next = read_varint(handshake, i);
	if (next.kind != VarintKind::Value || (next.value != 2 && next.value != 3))
		return {SniffKind::Done, {}};

	std::vector<std::uint8_t> start;
	state = frame(at, start);
	if (state != SniffKind::Name)
		return {state, {}};
	i = 0;
	packet_id = read_varint(start, i);
	if (packet_id.kind != VarintKind::Value || packet_id.value != 0)
		return {SniffKind::Done, {}};
	std::string name;
	if (!read_string(start, i, 16, name) || name.empty())
		return {SniffKind::Done, {}};
	return {SniffKind::Name, std::move(name)};
}

std::string lower(std::string value)
{
	std::transform(value.begin(), value.end(), value.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return value;
}

std::optional<boost::asio::ip::address_v4> lan_ip(boost::asio::io_context &io)
{
	boost::system::error_code ec;
	Udp::socket socket(io);
	socket.open(Udp::v4(), ec);
	if (ec)
		return std::nullopt;
	socket.connect(GROUP, ec);
	if (ec)
		return std::nullopt;
	const auto local = socket.local_endpoint(ec);
	if (ec || !local.address().is_v4() || local.address().is_unspecified() ||
			local.address().is_loopback())
		return std::nullopt;
	return local.address().to_v4();
}

std::vector<std::uint8_t> announcement(const std::string &motd, std::uint16_t port)
{
	std::string safe;
	safe.reserve(motd.size());
	for (const auto c : motd)
		safe.push_back(c == '[' ? '(' : c == ']' ? ')' : c);
	const auto message = "[MOTD]" + safe + "[/MOTD][AD]" + std::to_string(port) + "[/AD]";
	return {message.begin(), message.end()};
}
} // namespace

struct Lan::Shared
{
	struct Guests
	{
		std::size_t total = 0;
		std::unordered_map<std::string, std::size_t> by_address;

		bool take(const boost::asio::ip::address &ip)
		{
			const auto key = ip.to_string();
			auto &mine = by_address[key];
			if (total >= MAX_GUESTS || mine >= MAX_PER_GUEST) {
				if (mine == 0)
					by_address.erase(key);
				return false;
			}
			++mine;
			++total;
			return true;
		}

		void give_back(const boost::asio::ip::address &ip)
		{
			if (total)
				--total;
			const auto it = by_address.find(ip.to_string());
			if (it == by_address.end())
				return;
			if (--it->second == 0)
				by_address.erase(it);
		}
	};

	boost::asio::io_context io;
	std::unique_ptr<Tcp::acceptor> acceptor;
	std::atomic_bool stop{false};
	std::atomic_uint16_t server_port{0};
	mutable std::mutex here_mutex;
	std::unordered_set<std::string> here;
	std::mutex open_mutex;
	std::unordered_map<std::uint64_t, std::weak_ptr<Tcp::socket>> open;
	std::mutex guests_mutex;
	Guests guests;
};

std::string address(const std::string &host, std::uint16_t port)
{
	return port == DEFAULT_PORT ? host : host + ":" + std::to_string(port);
}

Lan::Lan(std::uint16_t port, std::shared_ptr<Shared> shared) :
		port_(port), shared_(std::move(shared))
{
}

std::unique_ptr<Lan> Lan::bind(std::optional<std::uint16_t> preferred, std::string &error)
{
	auto shared = std::make_shared<Shared>();
	shared->acceptor = std::make_unique<Tcp::acceptor>(shared->io);
	boost::system::error_code ec;
	const auto bind_port = [&](std::uint16_t port) {
		shared->acceptor->open(Tcp::v4(), ec);
		if (!ec)
			shared->acceptor->bind(Tcp::endpoint(Tcp::v4(), port), ec);
		if (ec)
			shared->acceptor->close();
		return !ec;
	};
	if (!preferred || !bind_port(*preferred)) {
		ec.clear();
		if (!bind_port(0)) {
			error = "Could not open the server to your network: " + ec.message();
			return nullptr;
		}
	}
	shared->acceptor->listen(boost::asio::socket_base::max_listen_connections, ec);
	if (ec) {
		error = "Could not open the server to your network: " + ec.message();
		return nullptr;
	}
	shared->acceptor->non_blocking(true, ec);
	if (ec) {
		error = "Could not open the server to your network: " + ec.message();
		return nullptr;
	}
	const auto port = shared->acceptor->local_endpoint(ec).port();
	if (ec) {
		error = ec.message();
		return nullptr;
	}
	auto lan = std::unique_ptr<Lan>(new Lan(port, shared));
	try {
		lan->accept_thread_ = std::thread([shared] {
			std::uint64_t next_id = 0;
			while (!shared->stop.load(std::memory_order_acquire)) {
				auto client = std::make_shared<Tcp::socket>(shared->io);
				boost::system::error_code accept_error;
				shared->acceptor->accept(*client, accept_error);
				if (accept_error) {
					std::this_thread::sleep_for(ACCEPT_PAUSE);
					continue;
				}
				client->non_blocking(false, accept_error);
				if (accept_error)
					continue;
				const auto server_port =
						shared->server_port.load(std::memory_order_acquire);
				if (server_port == 0)
					continue;
				boost::system::error_code endpoint_error;
				const auto peer = client->remote_endpoint(endpoint_error).address();
				if (endpoint_error)
					continue;
				const auto local = client->local_endpoint(endpoint_error).address();
				const bool here =
						!endpoint_error && (peer.is_loopback() || peer == local);
				const auto guest = here ? std::optional<boost::asio::ip::address>{}
										: std::optional<boost::asio::ip::address>{peer};
				if (guest) {
					std::lock_guard lock(shared->guests_mutex);
					if (!shared->guests.take(*guest))
						continue;
				}
				const auto id = ++next_id;
				try {
					std::thread([shared, client, server_port, here, guest, id] {
						auto relay = [&] {
							boost::system::error_code ec;
							Tcp::socket server(shared->io);
							server.connect(
									Tcp::endpoint(boost::asio::ip::address_v4::loopback(),
											server_port),
									ec);
							if (ec)
								return;
							client->set_option(Tcp::no_delay(true), ec);
							server.set_option(Tcp::no_delay(true), ec);
							client->non_blocking(true, ec);
							server.non_blocking(true, ec);
							if (ec)
								return;
							{
								std::lock_guard lock(shared->open_mutex);
								shared->open.emplace(id, client);
							}
							if (shared->stop.load(std::memory_order_acquire)) {
								boost::system::error_code ignored;
								client->shutdown(Tcp::socket::shutdown_both, ignored);
								return;
							}
							const auto pump = [&](Tcp::socket &from, Tcp::socket &to,
													  std::function<bool(
															  const std::uint8_t *,
															  std::size_t)>
															  sniff) {
								std::array<std::uint8_t, 16 * 1024> buffer{};
								bool sniffing = static_cast<bool>(sniff);
								auto last = Clock::now();
								for (;;) {
									boost::system::error_code read_error;
									const auto n = from.read_some(
											boost::asio::buffer(buffer), read_error);
									if (!read_error && n) {
										last = Clock::now();
										if (sniffing && sniff(buffer.data(), n))
											sniffing = false;
										std::size_t sent = 0;
										while (sent < n) {
											boost::system::error_code write_error;
											const auto wrote = to.write_some(
													boost::asio::buffer(
															buffer.data() + sent,
															n - sent),
													write_error);
											if (!write_error) {
												sent += wrote;
												last = Clock::now();
												continue;
											}
											if (write_error != boost::asio::error::
																	   would_block &&
													write_error !=
															boost::asio::error::try_again)
												return;
											if (Clock::now() - last > IDLE_TIMEOUT)
												return;
											std::this_thread::sleep_for(2ms);
										}
										continue;
									}
									if (read_error == boost::asio::error::interrupted)
										continue;
									if (read_error == boost::asio::error::would_block ||
											read_error == boost::asio::error::try_again) {
										const auto timeout =
												sniffing ? HELLO_TIMEOUT : IDLE_TIMEOUT;
										if (Clock::now() - last > timeout)
											break;
										std::this_thread::sleep_for(2ms);
										continue;
									}
									boost::system::error_code ignored;
									to.shutdown(Tcp::socket::shutdown_send, ignored);
									return;
								}
								boost::system::error_code ignored;
								from.shutdown(Tcp::socket::shutdown_both, ignored);
								to.shutdown(Tcp::socket::shutdown_both, ignored);
							};
							std::jthread back([&] { pump(server, *client, {}); });
							std::vector<std::uint8_t> seen;
							pump(*client, server,
									[&](const std::uint8_t *data, std::size_t size) {
										const auto room =
												SNIFF_LIMIT -
												std::min(seen.size(), SNIFF_LIMIT);
										const auto take = std::min(size, room);
										seen.insert(seen.end(), data, data + take);
										const auto result = login_name(seen);
										if (result.kind == SniffKind::Name) {
											if (here) {
												std::lock_guard lock(shared->here_mutex);
												shared->here.insert(lower(result.name));
											}
											return true;
										}
										return result.kind == SniffKind::Done ||
											   seen.size() >= SNIFF_LIMIT;
									});
							if (back.joinable())
								back.join();
						};
						try {
							relay();
						} catch (...) {
						}
						{
							std::lock_guard lock(shared->open_mutex);
							shared->open.erase(id);
						}
						if (guest) {
							std::lock_guard lock(shared->guests_mutex);
							shared->guests.give_back(*guest);
						}
					}).detach();
				} catch (...) {
					if (guest) {
						std::lock_guard lock(shared->guests_mutex);
						shared->guests.give_back(*guest);
					}
				}
			}
		});
	} catch (const std::exception &e) {
		error = e.what();
		return nullptr;
	}
	return lan;
}

Lan::~Lan()
{
	shared_->stop.store(true, std::memory_order_release);
	{
		std::lock_guard lock(shared_->open_mutex);
		for (const auto &[id, socket] : shared_->open) {
			(void)id;
			auto live = socket.lock();
			if (!live)
				continue;
			boost::system::error_code ignored;
			live->shutdown(Tcp::socket::shutdown_both, ignored);
		}
	}
	boost::system::error_code ignored;
	if (accept_thread_.joinable())
		accept_thread_.join();
	if (announce_thread_.joinable())
		announce_thread_.join();
	shared_->acceptor->close(ignored);
}

std::uint16_t Lan::port() const
{
	return port_;
}

bool Lan::open(std::uint16_t server_port, const std::string &motd,
		AddressCallback on_address, std::string &error)
{
	if (announce_thread_.joinable()) {
		error = "LAN announcement is already running";
		return false;
	}
	try {
		auto shared = shared_;
		const auto packet = announcement(motd, port_);
		announce_thread_ = std::thread([shared, packet, port = port_,
											   callback =
													   std::move(on_address)]() mutable {
			Udp::socket socket(shared->io);
			boost::system::error_code ec;
			socket.open(Udp::v4(), ec);
			const bool can_send = !ec;
			std::optional<std::optional<boost::asio::ip::address_v4>> known;
			bool failing = false;
			auto due = Clock::now();
			while (!shared->stop.load(std::memory_order_acquire)) {
				if (Clock::now() >= due) {
					due = Clock::now() + ANNOUNCE_EVERY;
					auto ip = lan_ip(shared->io);
					if (!known || *known != ip) {
						known = ip;
						if (callback)
							callback(ip ? std::optional<std::string>(
												  address(ip->to_string(), port))
										: std::nullopt);
					}
					if (can_send && ip) {
						boost::system::error_code send_error;
						socket.send_to(boost::asio::buffer(packet), GROUP, 0, send_error);
						if (send_error) {
							if (!failing)
								std::cerr
										<< "[live] Could not list the server under LAN worlds: "
										<< send_error.message() << '\n';
							failing = true;
						} else {
							failing = false;
						}
					}
				}
				std::this_thread::sleep_for(100ms);
			}
		});
		shared_->server_port.store(server_port, std::memory_order_release);
		return true;
	} catch (const std::exception &e) {
		error = e.what();
		return false;
	}
}

bool Lan::logged_in_here(const std::string &name) const
{
	std::lock_guard lock(shared_->here_mutex);
	return shared_->here.contains(lower(name));
}
} // namespace arnis::live::lan
