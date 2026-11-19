#include "child.h"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <utility>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace arnis::scale::child
{
bool adopt_process(void *native_process_handle, std::string *error)
{
#if defined(_WIN32)
	if (!native_process_handle) {
		if (error)
			*error = "AssignProcessToJobObject: invalid child process handle";
		return false;
	}
	struct Job
	{
		HANDLE handle = nullptr;
		std::string error;
	};
	// Intentionally retained until process exit: closing this job kills its children.
	static const Job job = [] {
		Job out;
		out.handle = CreateJobObjectW(nullptr, nullptr);
		if (!out.handle) {
			out.error = "CreateJobObject: " + std::to_string(GetLastError());
			return out;
		}
		JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
		limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
		if (!SetInformationJobObject(out.handle, JobObjectExtendedLimitInformation,
					&limits, sizeof(limits))) {
			out.error = "SetInformationJobObject: " + std::to_string(GetLastError());
			CloseHandle(out.handle);
			out.handle = nullptr;
		}
		return out;
	}();
	if (!job.handle) {
		if (error)
			*error = job.error;
		return false;
	}
	if (!AssignProcessToJobObject(
				job.handle, static_cast<HANDLE>(native_process_handle))) {
		if (error)
			*error = "AssignProcessToJobObject: " + std::to_string(GetLastError());
		return false;
	}
	return true;
#else
	(void)native_process_handle;
	(void)error;
	return true;
#endif
}

void watch_parent()
{
#if defined(__unix__)
	std::thread([] {
		char byte;
		while (std::cin.get(byte)) {
		}
		std::cerr << "The job's coordinator is gone; stopping this piece.\n";
		std::exit(1);
	}).detach();
#endif
}

void fail_for_test(std::size_t piece)
{
#if !defined(NDEBUG)
	const char *env_path = std::getenv("ARNIS_TEST_FAIL_PIECE");
	if (!env_path || !*env_path)
		return;
	const std::string path(env_path);
	std::ifstream input(path);
	if (!input)
		return;
	std::size_t target = 0;
	if (!(input >> target) || target != piece + 1)
		return;
	std::uint64_t delay_seconds = 0;
	input >> delay_seconds;
	input.close();
	if (std::remove(path.c_str()) != 0)
		return;
	std::thread([piece, delay_seconds] {
		std::this_thread::sleep_for(std::chrono::seconds(delay_seconds));
		std::cerr << "Error: piece " << piece + 1
				  << " stopped by ARNIS_TEST_FAIL_PIECE\n";
		std::exit(101);
	}).detach();
#else
	(void)piece;
#endif
}
} // namespace arnis::scale::child
