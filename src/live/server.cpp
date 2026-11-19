#include "server.h"

#include "rcon.h"
#include "../world_utils.h"

#include <boost/asio.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__APPLE__)
#include <cerrno>
#include <csignal>
#include <sys/types.h>
#include <sys/sysctl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#elif defined(__linux__)
#include <cerrno>
#include <csignal>
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#elif defined(__unix__)
#include <cerrno>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#else
#include <unistd.h>
#endif

namespace arnis::live::server
{
namespace
{
using Tcp = boost::asio::ip::tcp;

void set_error(std::string *out, std::string message)
{
	if (out)
		*out = std::move(message);
}

std::filesystem::path home_dir()
{
#if defined(_WIN32)
	if (const char *profile = std::getenv("USERPROFILE"); profile && *profile)
		return profile;
	if (const char *drive = std::getenv("HOMEDRIVE"))
		if (const char *path = std::getenv("HOMEPATH"))
			return std::filesystem::path(std::string(drive) + path);
#else
	if (const char *home = std::getenv("HOME"); home && *home)
		return home;
#endif
	return {};
}

bool bind_loopback(std::uint16_t port)
{
	boost::asio::io_context io;
	Tcp::acceptor acceptor(io);
	boost::system::error_code ec;
	acceptor.open(Tcp::v4(), ec);
	if (!ec)
		acceptor.bind(Tcp::endpoint(boost::asio::ip::address_v4::loopback(), port), ec);
	return !ec;
}

#if defined(_WIN32)
std::wstring widen_utf8(const std::string &text)
{
	if (text.empty())
		return {};
	const int length = MultiByteToWideChar(
			CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0);
	if (length <= 0)
		return std::wstring(text.begin(), text.end());
	std::wstring wide(static_cast<std::size_t>(length), L'\0');
	MultiByteToWideChar(
			CP_UTF8, 0, text.data(), static_cast<int>(text.size()), wide.data(), length);
	return wide;
}

std::wstring quote_windows_argument(const std::wstring &argument)
{
	std::wstring out(1, L'"');
	std::size_t slashes = 0;
	for (const wchar_t c : argument) {
		if (c == L'\\') {
			++slashes;
			continue;
		}
		if (c == L'"') {
			out.append(slashes * 2 + 1, L'\\');
			out.push_back(c);
		} else {
			out.append(slashes, L'\\');
			out.push_back(c);
		}
		slashes = 0;
	}
	out.append(slashes * 2, L'\\');
	out.push_back(L'"');
	return out;
}
#endif

bool write_text_file(
		const std::filesystem::path &path, const std::string &text, std::string *error)
{
	std::ofstream output(path, std::ios::binary | std::ios::trunc);
	if (!output ||
			!output.write(text.data(), static_cast<std::streamsize>(text.size()))) {
		set_error(error, "could not write " + path.filename().string());
		return false;
	}
	output.close();
	if (!output) {
		set_error(error, "could not write " + path.filename().string());
		return false;
	}
	keep_private(path);
	return true;
}
} // namespace

void StartupMonitor::observe_line(const std::string &line)
{
	if (line.find("Done (") != std::string::npos &&
			line.find("For help") != std::string::npos) {
		ready_.store(true, std::memory_order_release);
		return;
	}
	static const std::pair<const char *, const char *> failures[]{
			{"FAILED TO BIND TO PORT", "The server port is taken by another program."},
			{"You need to agree to the EULA",
					"The Minecraft EULA has not been accepted."},
			{"Session lock is already acquired",
					"The world is open in Minecraft. Leave it first."},
			{"session.lock: already locked",
					"The world is open in Minecraft. Leave it first."},
			{"UnsupportedClassVersionError", "This Java is too old for the server."},
			{"Failed to start the minecraft server", "The server failed to start."},
	};
	for (const auto &[needle, message] : failures) {
		if (line.find(needle) == std::string::npos)
			continue;
		std::lock_guard lock(failure_mutex_);
		if (!failure_)
			failure_ = message;
		return;
	}
}

bool StartupMonitor::ready() const
{
	return ready_.load(std::memory_order_acquire);
}

std::optional<std::string> StartupMonitor::failure() const
{
	std::lock_guard lock(failure_mutex_);
	return failure_;
}

RecentLog::RecentLog(std::size_t capacity) : capacity_(capacity)
{
}

void RecentLog::append(std::string line)
{
	if (capacity_ == 0)
		return;
	std::lock_guard lock(mutex_);
	if (lines_.size() == capacity_)
		lines_.pop_front();
	lines_.push_back(std::move(line));
}

std::string RecentLog::tail(std::size_t count) const
{
	std::lock_guard lock(mutex_);
	const auto begin = lines_.size() - std::min(lines_.size(), count);
	std::string result;
	for (std::size_t i = begin; i < lines_.size(); ++i) {
		if (!result.empty())
			result.push_back('\n');
		result += lines_[i];
	}
	return result;
}

struct Server::State
{
	StartupMonitor startup;
	RecentLog log;
	std::thread stdout_reader;
	std::thread stderr_reader;
#if defined(_WIN32)
	HANDLE process = nullptr;
	HANDLE stdin_write = nullptr;
	HANDLE stdout_read = nullptr;
	HANDLE stderr_read = nullptr;
	bool exited = false;
	DWORD exit_code = STILL_ACTIVE;
#else
	pid_t process = -1;
	int stdin_write = -1;
	int stdout_read = -1;
	int stderr_read = -1;
	bool exited = false;
	int wait_status = 0;
#endif

	~State()
	{
		if (running())
			terminate();
		if (stdout_reader.joinable())
			stdout_reader.join();
		if (stderr_reader.joinable())
			stderr_reader.join();
#if defined(_WIN32)
		close_handle(stdin_write);
		close_handle(stdout_read);
		close_handle(stderr_read);
		close_handle(process);
#else
		close_fd(stdin_write);
		close_fd(stdout_read);
		close_fd(stderr_read);
#endif
	}

	static void consume(State *self, int stream)
	{
		std::string pending;
		std::array<char, 4096> buffer{};
#if defined(_WIN32)
		HANDLE handle = stream == 1 ? self->stdout_read : self->stderr_read;
		for (;;) {
			DWORD count = 0;
			if (!ReadFile(handle, buffer.data(), static_cast<DWORD>(buffer.size()),
						&count, nullptr) ||
					count == 0)
				break;
			pending.append(buffer.data(), count);
#else
		const int fd = stream == 1 ? self->stdout_read : self->stderr_read;
		for (;;) {
			const auto count = ::read(fd, buffer.data(), buffer.size());
			if (count == 0)
				break;
			if (count < 0) {
				if (errno == EINTR)
					continue;
				break;
			}
			pending.append(buffer.data(), static_cast<std::size_t>(count));
#endif
			std::size_t newline = 0;
			while ((newline = pending.find('\n')) != std::string::npos) {
				std::string line = pending.substr(0, newline);
				if (!line.empty() && line.back() == '\r')
					line.pop_back();
				self->startup.observe_line(line);
				self->log.append(std::move(line));
				pending.erase(0, newline + 1);
			}
		}
		if (!pending.empty()) {
			if (pending.back() == '\r')
				pending.pop_back();
			self->startup.observe_line(pending);
			self->log.append(std::move(pending));
		}
	}

	bool running()
	{
		if (exited)
			return false;
#if defined(_WIN32)
		if (!process)
			return false;
		const DWORD result = WaitForSingleObject(process, 0);
		if (result == WAIT_TIMEOUT)
			return true;
		if (result == WAIT_OBJECT_0) {
			GetExitCodeProcess(process, &exit_code);
			exited = true;
			return false;
		}
		exit_code = GetLastError();
		exited = true;
		return false;
#else
		if (process <= 0)
			return false;
		pid_t result;
		do {
			result = ::waitpid(process, &wait_status, WNOHANG);
		} while (result < 0 && errno == EINTR);
		if (result == 0 || (result < 0 && errno == ECHILD))
			return result == 0;
		exited = true;
		return false;
#endif
	}

	std::string status_text() const
	{
#if defined(_WIN32)
		return "exit code " + std::to_string(exit_code);
#else
		if (WIFEXITED(wait_status))
			return "exit code " + std::to_string(WEXITSTATUS(wait_status));
		if (WIFSIGNALED(wait_status))
			return "signal " + std::to_string(WTERMSIG(wait_status));
		return "unknown status";
#endif
	}

	void write_console_stop()
	{
		static constexpr char command[] = "stop\n";
#if defined(_WIN32)
		if (stdin_write) {
			DWORD written = 0;
			WriteFile(stdin_write, command, sizeof(command) - 1, &written, nullptr);
			FlushFileBuffers(stdin_write);
		}
#else
		if (stdin_write >= 0) {
			std::size_t offset = 0;
			while (offset < sizeof(command) - 1) {
				const auto n = ::write(
						stdin_write, command + offset, sizeof(command) - 1 - offset);
				if (n > 0) {
					offset += static_cast<std::size_t>(n);
					continue;
				}
				if (n < 0 && errno == EINTR)
					continue;
				break;
			}
		}
#endif
	}

	void terminate()
	{
#if defined(_WIN32)
		if (process && !exited) {
			TerminateProcess(process, 1);
			WaitForSingleObject(process, INFINITE);
			GetExitCodeProcess(process, &exit_code);
			exited = true;
		}
#else
		if (process > 0 && !exited) {
			::kill(process, SIGKILL);
			while (::waitpid(process, &wait_status, 0) < 0 && errno == EINTR) {
			}
			exited = true;
		}
#endif
	}

	bool launch(const std::filesystem::path &working_dir, const Launch &options,
			std::string *error)
	{
		const auto args = java_arguments(options, system_total_memory_bytes());
#if defined(_WIN32)
		SECURITY_ATTRIBUTES security{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
		HANDLE child_stdin_read = nullptr, child_stdout_write = nullptr,
			   child_stderr_write = nullptr;
		if (!CreatePipe(&child_stdin_read, &stdin_write, &security, 0) ||
				!CreatePipe(&stdout_read, &child_stdout_write, &security, 0) ||
				!CreatePipe(&stderr_read, &child_stderr_write, &security, 0)) {
			if (error)
				*error = "Could not create Java pipes: " + std::to_string(GetLastError());
			close_handle(child_stdin_read);
			close_handle(child_stdout_write);
			close_handle(child_stderr_write);
			close_handle(stdin_write);
			close_handle(stdout_read);
			close_handle(stderr_read);
			return false;
		}
		SetHandleInformation(stdin_write, HANDLE_FLAG_INHERIT, 0);
		SetHandleInformation(stdout_read, HANDLE_FLAG_INHERIT, 0);
		SetHandleInformation(stderr_read, HANDLE_FLAG_INHERIT, 0);
		STARTUPINFOW startup_info{};
		startup_info.cb = sizeof(startup_info);
		startup_info.dwFlags = STARTF_USESTDHANDLES;
		startup_info.hStdInput = child_stdin_read;
		startup_info.hStdOutput = child_stdout_write;
		startup_info.hStdError = child_stderr_write;
		std::wstring command = quote_windows_argument(widen_utf8(options.java.string()));
		for (const auto &arg : args) {
			command.push_back(L' ');
			command += quote_windows_argument(widen_utf8(arg));
		}
		PROCESS_INFORMATION info{};
		const auto application = options.java.wstring();
		const auto cwd = working_dir.wstring();
		const BOOL created = CreateProcessW(application.c_str(), command.data(), nullptr,
				nullptr, TRUE, CREATE_NEW_PROCESS_GROUP | CREATE_NO_WINDOW, nullptr,
				cwd.c_str(), &startup_info, &info);
		close_handle(child_stdin_read);
		close_handle(child_stdout_write);
		close_handle(child_stderr_write);
		if (!created) {
			if (error)
				*error = "Could not start Java for the server: " +
						 std::to_string(GetLastError());
			return false;
		}
		process = info.hProcess;
		CloseHandle(info.hThread);
#else
		int in_pipe[2]{-1, -1}, out_pipe[2]{-1, -1}, err_pipe[2]{-1, -1};
		auto close_pipe = [](int(&p)[2]) {
			for (int &fd : p)
				close_fd(fd);
		};
		if (::pipe(in_pipe) != 0 || ::pipe(out_pipe) != 0 || ::pipe(err_pipe) != 0) {
			if (error)
				*error = "Could not create Java pipes: " +
						 std::string(std::strerror(errno));
			close_pipe(in_pipe);
			close_pipe(out_pipe);
			close_pipe(err_pipe);
			return false;
		}
		std::vector<std::string> storage;
		storage.reserve(args.size() + 1);
		storage.push_back(options.java.string());
		storage.insert(storage.end(), args.begin(), args.end());
		std::vector<char *> argv;
		argv.reserve(storage.size() + 1);
		for (auto &arg : storage)
			argv.push_back(arg.data());
		argv.push_back(nullptr);
		const std::string cwd = working_dir.string();
		const pid_t pid = ::fork();
		if (pid < 0) {
			if (error)
				*error = "Could not start Java for the server: " +
						 std::string(std::strerror(errno));
			close_pipe(in_pipe);
			close_pipe(out_pipe);
			close_pipe(err_pipe);
			return false;
		}
		if (pid == 0) {
			::setpgid(0, 0);
			if (::chdir(cwd.c_str()) != 0)
				::_exit(126);
			::dup2(in_pipe[0], STDIN_FILENO);
			::dup2(out_pipe[1], STDOUT_FILENO);
			::dup2(err_pipe[1], STDERR_FILENO);
			close_pipe(in_pipe);
			close_pipe(out_pipe);
			close_pipe(err_pipe);
			::execvp(argv.front(), argv.data());
			const char message[] = "Could not execute Java.\n";
			::write(STDERR_FILENO, message, sizeof(message) - 1);
			::_exit(127);
		}
		process = pid;
		close_fd(in_pipe[0]);
		close_fd(out_pipe[1]);
		close_fd(err_pipe[1]);
		stdin_write = in_pipe[1];
		stdout_read = out_pipe[0];
		stderr_read = err_pipe[0];
#endif
		stdout_reader = std::thread(&State::consume, this, 1);
		stderr_reader = std::thread(&State::consume, this, 2);
		return true;
	}

#if defined(_WIN32)
	static void close_handle(HANDLE &handle)
	{
		if (handle && handle != INVALID_HANDLE_VALUE)
			CloseHandle(handle);
		handle = nullptr;
	}
#else
	static void close_fd(int &fd)
	{
		if (fd >= 0)
			::close(fd);
		fd = -1;
	}
#endif
};

Server::Server(std::unique_ptr<State> state) : state_(std::move(state))
{
}

Server::~Server()
{
	if (state_)
		stop(nullptr, std::chrono::seconds(60));
}

std::unique_ptr<Server> Server::start(
		const std::filesystem::path &dir, const Launch &launch, std::string *error)
{
	std::error_code ec;
	std::filesystem::create_directories(dir, ec);
	if (ec) {
		set_error(error, ec.message());
		return nullptr;
	}
	if (!eula_accepted(dir)) {
		set_error(error, "The Minecraft EULA has not been accepted.");
		return nullptr;
	}
	if (!prepare_server_files(dir, launch, error))
		return nullptr;
	auto state = std::make_unique<State>();
	if (!state->launch(dir, launch, error))
		return nullptr;
	return std::unique_ptr<Server>(new Server(std::move(state)));
}

bool Server::wait_ready(std::chrono::milliseconds timeout,
		const std::function<bool()> &cancel, std::string *error)
{
	const auto start = std::chrono::steady_clock::now();
	for (;;) {
		if (state_->startup.ready())
			return true;
		if (const auto failure = state_->startup.failure()) {
			set_error(error, *failure);
			return false;
		}
		if (!state_->running()) {
			set_error(error, "The server stopped while starting (" +
									 state_->status_text() + "). Last lines:\n" +
									 state_->log.tail(12));
			return false;
		}
		if (cancel && cancel()) {
			set_error(error, "Cancelled");
			return false;
		}
		if (std::chrono::steady_clock::now() - start > timeout) {
			set_error(error, "The server took too long to start.");
			return false;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

bool Server::running()
{
	return state_ && state_->running();
}

std::string Server::tail(std::size_t count) const
{
	return state_ ? state_->log.tail(count) : std::string{};
}

void Server::stop(rcon::Rcon *rcon, std::chrono::milliseconds timeout)
{
	if (!state_ || !state_->running())
		return;
	bool sent = false;
	if (rcon) {
		std::string error;
		sent = rcon->cmd("stop", error).has_value();
	}
	if (!sent)
		state_->write_console_stop();
	const auto deadline = std::chrono::steady_clock::now() + timeout;
	while (std::chrono::steady_clock::now() < deadline) {
		if (!state_->running())
			return;
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
	state_->terminate();
}

std::filesystem::path server_dir()
{
	if (const char *dir = std::getenv("ARNIS_LIVE_SERVER_DIR"); dir && *dir)
		return dir;
	std::filesystem::path base;
#if defined(_WIN32)
	if (const char *local = std::getenv("LOCALAPPDATA"); local && *local)
		base = local;
#elif defined(__APPLE__)
	if (const auto home = home_dir(); !home.empty())
		base = home / "Library" / "Application Support";
#else
	if (const char *xdg = std::getenv("XDG_DATA_HOME"); xdg && *xdg)
		base = xdg;
	else if (const auto home = home_dir(); !home.empty())
		base = home / ".local" / "share";
#endif
	if (base.empty())
		base = home_dir();
	if (base.empty())
		base = std::filesystem::temp_directory_path();
	return base / "arnis" / "live-server";
}

bool eula_accepted(const std::filesystem::path &dir)
{
	std::ifstream input(dir / "eula.txt");
	std::string line;
	while (std::getline(input, line)) {
		const auto first = line.find_first_not_of(" \t\r\n");
		if (first == std::string::npos)
			continue;
		const auto last = line.find_last_not_of(" \t\r\n");
		line = line.substr(first, last - first + 1);
		std::transform(line.begin(), line.end(), line.begin(),
				[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		if (line == "eula=true")
			return true;
	}
	return false;
}

bool accept_eula(const std::filesystem::path &dir, std::string *error)
{
	std::error_code ec;
	std::filesystem::create_directories(dir, ec);
	if (ec) {
		set_error(error, ec.message());
		return false;
	}
	const auto text = std::string("# Agreed in Arnis to the Minecraft EULA (") +
					  EULA_URL + ")\neula=true\n";
	std::ofstream output(dir / "eula.txt", std::ios::binary | std::ios::trunc);
	if (!output ||
			!output.write(text.data(), static_cast<std::streamsize>(text.size()))) {
		set_error(error, "could not write eula.txt");
		return false;
	}
	output.close();
	if (!output) {
		set_error(error, "could not write eula.txt");
		return false;
	}
	return true;
}

std::uint16_t free_port(std::optional<std::uint16_t> preferred)
{
	if (preferred && bind_loopback(*preferred))
		return *preferred;
	boost::asio::io_context io;
	Tcp::acceptor acceptor(io);
	boost::system::error_code ec;
	acceptor.open(Tcp::v4(), ec);
	if (!ec)
		acceptor.bind(Tcp::endpoint(boost::asio::ip::address_v4::loopback(), 0), ec);
	if (!ec) {
		const auto endpoint = acceptor.local_endpoint(ec);
		if (!ec)
			return endpoint.port();
	}
	return 25565;
}

std::uint16_t free_port_besides(std::uint16_t taken)
{
	boost::asio::io_context io;
	Tcp::acceptor held(io);
	boost::system::error_code ec;
	held.open(Tcp::v4(), ec);
	if (!ec)
		held.bind(Tcp::endpoint(boost::asio::ip::address_v4::loopback(), taken), ec);
	return free_port(std::nullopt);
}

bool port_free(std::uint16_t port)
{
	return bind_loopback(port);
}

std::string motd(const std::string &world_name)
{
	return "Arnis Live \xC2\xB7 " + world_name;
}

std::uint64_t system_total_memory_bytes()
{
#if defined(_WIN32)
	MEMORYSTATUSEX status{};
	status.dwLength = sizeof(status);
	if (GlobalMemoryStatusEx(&status))
		return status.ullTotalPhys;
#elif defined(__APPLE__)
	std::uint64_t bytes = 0;
	size_t size = sizeof(bytes);
	if (sysctlbyname("hw.memsize", &bytes, &size, nullptr, 0) == 0)
		return bytes;
#elif defined(__linux__)
	struct sysinfo info
	{
	};
	if (sysinfo(&info) == 0)
		return static_cast<std::uint64_t>(info.totalram) * info.mem_unit;
#elif defined(_SC_PHYS_PAGES) && defined(_SC_PAGESIZE)
	const long pages = sysconf(_SC_PHYS_PAGES);
	const long page_size = sysconf(_SC_PAGESIZE);
	if (pages > 0 && page_size > 0)
		return static_cast<std::uint64_t>(pages) * static_cast<std::uint64_t>(page_size);
#endif
	return 4ULL * 1024 * 1024 * 1024;
}

std::uint64_t heap_limit_mb(std::uint64_t total_memory_bytes)
{
	const std::uint64_t total_mb = total_memory_bytes / (1024 * 1024);
	return std::clamp<std::uint64_t>(total_mb / 4, 1024, 4096);
}

std::vector<std::string> java_arguments(
		const Launch &launch, std::uint64_t total_memory_bytes)
{
	return {"-Xmx" + std::to_string(heap_limit_mb(total_memory_bytes)) + "M", "-Xms512M",
			"-jar", launch.jar.string(), "--nogui", "--universe",
			launch.saves_dir.string(), "--world", launch.world_name};
}

bool prepare_server_files(
		const std::filesystem::path &dir, const Launch &launch, std::string *error)
{
	std::error_code ec;
	std::filesystem::create_directories(dir, ec);
	if (ec) {
		set_error(error, ec.message());
		return false;
	}
	if (!write_properties(dir, launch, error))
		return false;
	// All worlds share this server folder; permissions are derived from the world on each run.
	if (!write_text_file(dir / "ops.json", "[]", error))
		return false;
	// All players connect through loopback, so an IP ban would lock out every player.
	return write_text_file(dir / "banned-ips.json", "[]", error);
}

bool write_properties(
		const std::filesystem::path &dir, const Launch &launch, std::string *error)
{
	std::error_code ec;
	std::filesystem::create_directories(dir, ec);
	if (ec) {
		set_error(error, ec.message());
		return false;
	}
	const std::vector<std::pair<std::string, std::string>> properties{
			{"server-ip", "127.0.0.1"},
			{"server-port", std::to_string(launch.port)},
			{"level-name", launch.world_name},
			{"motd", motd(launch.world_name)},
			{"online-mode", "true"},
			{"white-list", "false"},
			{"enforce-whitelist", "false"},
			{"enable-rcon", "true"},
			{"rcon.port", std::to_string(launch.rcon_port)},
			{"rcon.password", launch.rcon_password},
			{"broadcast-rcon-to-ops", "false"},
			{"broadcast-console-to-ops", "false"},
			{"view-distance", std::to_string(launch.view_distance)},
			{"simulation-distance", std::to_string(std::min(launch.view_distance, 10u))},
			{"gamemode", launch.rules.gamemode},
			{"difficulty", launch.rules.difficulty},
			{"hardcore", launch.rules.hardcore ? "true" : "false"},
			{"force-gamemode", "false"},
			{"allow-flight", "true"},
			{"spawn-protection", "0"},
			{"enable-command-block", "true"},
			{"max-players", "8"},
			{"pause-when-empty-seconds", "-1"},
			{"log-ips", "false"},
			{"sync-chunk-writes", "true"},
	};
	std::string text = "# Written by Arnis for live mode on every start.\n";
	for (const auto &[key, value] : properties)
		text += key + "=" + value + "\n";
	const std::vector<std::uint8_t> bytes(text.begin(), text.end());
	const auto path = dir / "server.properties";
	if (!world_utils::replace_file_atomically(path, bytes)) {
		set_error(error, "could not write server.properties");
		return false;
	}
	keep_private(path);
	return true;
}

void keep_private(const std::filesystem::path &path)
{
#if defined(__unix__) || defined(__APPLE__)
	std::error_code ec;
	std::filesystem::permissions(path,
			std::filesystem::perms::owner_read | std::filesystem::perms::owner_write,
			std::filesystem::perm_options::replace, ec);
#else
	(void)path;
#endif
}
} // namespace arnis::live::server
