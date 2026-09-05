#include "slam_remote/diagnostics/pangolin_live_diagnostics.hpp"

#include <iomanip>
#include <sstream>
#include <string>

#include <pangolin/display/default_font.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/widgets.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/handler/handler.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>

namespace slam_remote::diagnostics {
namespace {

constexpr char kWindowTitle[] = "SLAM Sender Diagnostics";

std::string TrackingStateName(slam::TrackingState state) {
    switch (state) {
        case slam::TrackingState::kInitializing:
            return "initializing";
        case slam::TrackingState::kTracking:
            return "tracking";
        case slam::TrackingState::kLost:
            return "lost";
        case slam::TrackingState::kRelocalizing:
            return "relocalizing";
    }
    return "unknown";
}

std::string Decimal(double value, int precision) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(precision) << value;
    return output.str();
}

}  // namespace

void RunPangolinLiveDiagnostics(LiveDiagnosticsStore& store,
                                const std::function<void()>& request_stop) {
    pangolin::CreateWindowAndBind(kWindowTitle, 1280, 720);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.04F, 0.04F, 0.05F, 1.0F);

    const int panel_width = 24 * pangolin::default_font().MaxWidth();
    pangolin::CreatePanel("ui")
        .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(panel_width));

    pangolin::Var<bool> stop_button("ui.Stop", false, false);
    pangolin::Var<std::string> tracking_state("ui.Tracking", "initializing");
    pangolin::Var<std::string> frames("ui.Frames", "0");
    pangolin::Var<std::string> poses("ui.Poses", "0");
    pangolin::Var<std::string> pointcloud_deltas("ui.Pointcloud_deltas", "0");
    pangolin::Var<std::string> active_points("ui.Active_points", "0");
    pangolin::Var<std::string> dropped_frames("ui.Dropped_frames", "0");
    pangolin::Var<std::string> input_fps("ui.Input_FPS", "0.00");
    pangolin::Var<std::string> processed_fps("ui.Processed_FPS", "0.00");
    pangolin::Var<std::string> mean_tracking_ms("ui.Mean_tracking_ms", "0.00");
    pangolin::Var<std::string> status("ui.Status", "starting");

    auto camera = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.01, 1000),
        pangolin::ModelViewLookAt(0.0, -1.0, -3.0, 0.0, 0.0, 0.0, pangolin::AxisY));
    auto& image_view = pangolin::Display("camera-image")
                           .SetBounds(0.0, 1.0, pangolin::Attach::Pix(panel_width), 0.55);
    auto& point_view =
        pangolin::Display("active-map-points")
            .SetBounds(0.0, 1.0, 0.55, 1.0)
            .SetHandler(new pangolin::Handler3D(camera));

    pangolin::GlTexture image_texture;
    std::uint32_t texture_width = 0;
    std::uint32_t texture_height = 0;

    while (true) {
        const auto snapshot = store.Snapshot();
        tracking_state = TrackingStateName(snapshot.stats.tracking_state);
        frames = std::to_string(snapshot.stats.frames);
        poses = std::to_string(snapshot.stats.poses);
        pointcloud_deltas = std::to_string(snapshot.stats.pointcloud_deltas);
        active_points = std::to_string(snapshot.points ? snapshot.points->size() : 0);
        dropped_frames = std::to_string(snapshot.stats.dropped_frames);
        input_fps = Decimal(snapshot.stats.input_fps, 2);
        processed_fps = Decimal(snapshot.stats.processed_fps, 2);
        mean_tracking_ms = Decimal(snapshot.stats.mean_tracking_ms, 2);
        status = snapshot.error.empty() ? (snapshot.finished ? "finished" : "running")
                                        : snapshot.error;

        if (pangolin::Pushed(stop_button) || pangolin::ShouldQuit()) {
            request_stop();
            break;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (snapshot.image) {
            if (!image_texture.IsValid() || texture_width != snapshot.image->width ||
                texture_height != snapshot.image->height) {
                image_texture.Reinitialise(snapshot.image->width, snapshot.image->height, GL_RGB8,
                                           true, 0, GL_RGB, GL_UNSIGNED_BYTE);
                texture_width = snapshot.image->width;
                texture_height = snapshot.image->height;
            }
            image_texture.Upload(snapshot.image->rgb_pixels.data(), GL_RGB, GL_UNSIGNED_BYTE);
            image_view.Activate();
            glColor3f(1.0F, 1.0F, 1.0F);
            image_texture.RenderToViewportFlipY();
        }

        point_view.Activate(camera);
        pangolin::glDrawAxis(0.5F);
        if (snapshot.points) {
            glPointSize(2.0F);
            glColor3f(0.2F, 0.85F, 1.0F);
            glBegin(GL_POINTS);
            for (const auto& point : *snapshot.points) {
                glVertex3d(point.position_metres[0], point.position_metres[1],
                           point.position_metres[2]);
            }
            glEnd();
        }

        pangolin::FinishFrame();
        if (snapshot.finished) break;
    }
    pangolin::DestroyWindow(kWindowTitle);
}

}  // namespace slam_remote::diagnostics
