#pragma once

#include <functional>

#include "slam_remote/diagnostics/live_diagnostics.hpp"

namespace slam_remote::diagnostics {

/// Runs the macOS Pangolin event loop on the calling thread until the producer
/// finishes or the user requests a stop.
void RunPangolinLiveDiagnostics(LiveDiagnosticsStore& store,
                                const std::function<void()>& request_stop);

}  // namespace slam_remote::diagnostics
