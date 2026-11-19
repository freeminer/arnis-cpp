#pragma once

#include <cstddef>
#include <string>

namespace arnis::scale::child
{
// On Windows, associates a spawned child with the process-lifetime job object
// configured to terminate all assigned children when this process exits. On
// other platforms there is no additional adoption step.
bool adopt_process(void *native_process_handle, std::string *error = nullptr);

// In a worker process, exits when its coordinator closes the inherited stdin
// pipe. The launcher must create the child with piped stdin and close its write
// end when it no longer owns the worker.
void watch_parent();

// Debug-only failure injection used by the piece retry path. `piece` is zero
// based; the control file names pieces starting at one.
void fail_for_test(std::size_t piece);
} // namespace arnis::scale::child
