#pragma once

namespace slam_remote::diagnostics {

/// Removes a trailing --diagnostics option from argc and reports whether it
/// was present. The producer's existing positional CLI remains unchanged.
bool ConsumeDiagnosticsOption(int& argc, char** argv);

}  // namespace slam_remote::diagnostics
