#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CoreMedia.h>
#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>

#include "slam_remote/camera/macos_camera_source.hpp"

#include <atomic>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "slam_remote/camera/bounded_frame_queue.hpp"

namespace slam_remote::camera {
namespace {

std::string ToString(NSString* value) {
    if (value == nil) {
        return {};
    }
    const char* utf8 = value.UTF8String;
    return utf8 == nullptr ? std::string{} : std::string(utf8);
}

NSArray<AVCaptureDeviceType>* CameraDeviceTypes() {
    return @[ AVCaptureDeviceTypeBuiltInWideAngleCamera, AVCaptureDeviceTypeExternal ];
}

AVCaptureDevice* FindDevice(const std::string& device_id) {
    AVCaptureDeviceDiscoverySession* discovery = [AVCaptureDeviceDiscoverySession
        discoverySessionWithDeviceTypes:CameraDeviceTypes()
                           mediaType:AVMediaTypeVideo
                            position:AVCaptureDevicePositionUnspecified];
    for (AVCaptureDevice* device in discovery.devices) {
        if (device_id.empty() || ToString(device.uniqueID) == device_id) {
            return device;
        }
    }
    return nil;
}

bool SupportsFps(AVCaptureDeviceFormat* format, std::uint32_t fps) {
    for (AVFrameRateRange* range in format.videoSupportedFrameRateRanges) {
        if (range.minFrameRate <= fps && fps <= range.maxFrameRate) {
            return true;
        }
    }
    return false;
}

AVCaptureDeviceFormat* FindFormat(AVCaptureDevice* device, std::uint32_t width,
                                  std::uint32_t height, std::uint32_t fps) {
    for (AVCaptureDeviceFormat* format in device.formats) {
        const CMVideoDimensions dimensions =
            CMVideoFormatDescriptionGetDimensions(format.formatDescription);
        if (dimensions.width == static_cast<std::int32_t>(width) &&
            dimensions.height == static_cast<std::int32_t>(height) && SupportsFps(format, fps)) {
            return format;
        }
    }
    return nil;
}

}  // namespace

void DeliverSampleBuffer(void* context, CMSampleBufferRef sample_buffer) noexcept;
void DeliverDroppedBuffer(void* context) noexcept;

}  // namespace slam_remote::camera

@interface SlamVideoDelegate : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
@property(nonatomic, assign) void* context;
@end

@implementation SlamVideoDelegate
- (void)captureOutput:(AVCaptureOutput*)output
    didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
           fromConnection:(AVCaptureConnection*)connection {
    (void)output;
    (void)connection;
    slam_remote::camera::DeliverSampleBuffer(self.context, sampleBuffer);
}
- (void)captureOutput:(AVCaptureOutput*)output
    didDropSampleBuffer:(CMSampleBufferRef)sampleBuffer
          fromConnection:(AVCaptureConnection*)connection {
    (void)output;
    (void)sampleBuffer;
    (void)connection;
    slam_remote::camera::DeliverDroppedBuffer(self.context);
}
@end

namespace slam_remote::camera {

class MacosCameraSource::Impl final {
   public:
    explicit Impl(MacosCameraConfig config)
        : config_(std::move(config)),
          queue_(config_.queue_capacity == 0 ? 1 : config_.queue_capacity) {}

    ~Impl() { Stop(); }

    StartResult Start() {
        if (started_) {
            return StartResult::Failure("macOS camera source is already started");
        }
        if (config_.width == 0 || config_.height == 0 || config_.fps == 0) {
            return StartResult::Failure("camera width, height, and FPS must be positive");
        }
        if (config_.queue_capacity == 0) {
            return StartResult::Failure("camera queue capacity must be positive");
        }
        if (const auto error = config_.calibration.Validate(); error.has_value()) {
            return StartResult::Failure(*error);
        }
        if (config_.calibration.width != config_.width ||
            config_.calibration.height != config_.height) {
            return StartResult::Failure("camera mode dimensions do not match calibration");
        }
        if (GetCameraAuthorization() != CameraAuthorization::kAuthorized) {
            return StartResult::Failure(
                "camera access is not authorized; run macos_camera_dump --request-permission");
        }

        device_ = FindDevice(config_.device_id);
        if (device_ == nil) {
            return StartResult::Failure(config_.device_id.empty()
                                            ? "no macOS video capture device is available"
                                            : "configured macOS camera device was not found");
        }

        AVCaptureDeviceFormat* format =
            FindFormat(device_, config_.width, config_.height, config_.fps);
        if (format == nil) {
            device_ = nil;
            return StartResult::Failure("camera does not support requested width, height, and FPS");
        }

        NSError* error = nil;
        if (![device_ lockForConfiguration:&error]) {
            const std::string message = "cannot configure camera: " + ToString(error.localizedDescription);
            device_ = nil;
            return StartResult::Failure(message);
        }
        @try {
            device_.activeFormat = format;
            const CMTime duration = CMTimeMake(1, static_cast<std::int32_t>(config_.fps));
            device_.activeVideoMinFrameDuration = duration;
            device_.activeVideoMaxFrameDuration = duration;
        } @catch (NSException* exception) {
            [device_ unlockForConfiguration];
            const std::string message = "camera mode negotiation failed: " + ToString(exception.reason);
            device_ = nil;
            return StartResult::Failure(message);
        }
        [device_ unlockForConfiguration];

        session_ = [[AVCaptureSession alloc] init];
        input_ = [AVCaptureDeviceInput deviceInputWithDevice:device_ error:&error];
        if (input_ == nil || ![session_ canAddInput:input_]) {
            const std::string message = "cannot create camera input: " + ToString(error.localizedDescription);
            ReleaseCaptureObjects();
            return StartResult::Failure(message);
        }
        [session_ addInput:input_];

        output_ = [[AVCaptureVideoDataOutput alloc] init];
        output_.alwaysDiscardsLateVideoFrames = YES;
        output_.videoSettings = @{
            (NSString*)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA),
            (NSString*)kCVPixelBufferWidthKey : @(config_.width),
            (NSString*)kCVPixelBufferHeightKey : @(config_.height),
        };
        if (![session_ canAddOutput:output_]) {
            ReleaseCaptureObjects();
            return StartResult::Failure("capture session cannot add BGRA video output");
        }
        [session_ addOutput:output_];

        callback_queue_ = dispatch_queue_create("slam.remote.camera.capture", DISPATCH_QUEUE_SERIAL);
        delegate_ = [[SlamVideoDelegate alloc] init];
        delegate_.context = this;
        [output_ setSampleBufferDelegate:delegate_ queue:callback_queue_];

        queue_.Reset();
        sequence_validator_.Reset();
        cancelled_.store(false);
        next_frame_id_.store(0);
        capture_dropped_frames_.store(0);
        timestamp_origin_set_ = false;
        info_ = {ToString(device_.uniqueID), ToString(device_.localizedName), config_.width,
                 config_.height, config_.fps};
        [session_ startRunning];
        if (!session_.running) {
            ReleaseCaptureObjects();
            return StartResult::Failure("AVFoundation capture session did not start");
        }
        started_ = true;
        return StartResult::Success();
    }

    FrameResult NextFrame(std::chrono::milliseconds timeout) {
        if (!started_) {
            return FrameResult::WithoutFrame(FrameStatus::kFatalError,
                                             "macOS camera source is not started");
        }
        if (cancelled_.load()) {
            return FrameResult::WithoutFrame(FrameStatus::kCancelled);
        }
        if (device_ == nil || !device_.connected) {
            return FrameResult::WithoutFrame(FrameStatus::kRecoverableError,
                                             "macOS camera device disconnected");
        }
        auto frame = queue_.WaitPop(timeout);
        if (!frame.has_value()) {
            return cancelled_.load()
                       ? FrameResult::WithoutFrame(FrameStatus::kCancelled)
                       : FrameResult::WithoutFrame(FrameStatus::kTimeout);
        }
        return FrameResult::Available(std::move(*frame));
    }

    void RequestStop() noexcept {
        cancelled_.store(true);
        queue_.Cancel();
    }

    void Stop() noexcept {
        RequestStop();
        if (session_ != nil && session_.running) {
            [session_ stopRunning];
        }
        ReleaseCaptureObjects();
        started_ = false;
    }

    void OnSampleBuffer(CMSampleBufferRef sample_buffer) noexcept {
        if (cancelled_.load()) {
            return;
        }
        CVImageBufferRef image = CMSampleBufferGetImageBuffer(sample_buffer);
        if (image == nullptr || CVPixelBufferLockBaseAddress(image, kCVPixelBufferLock_ReadOnly) !=
                                    kCVReturnSuccess) {
            return;
        }

        const auto width = CVPixelBufferGetWidth(image);
        const auto height = CVPixelBufferGetHeight(image);
        const auto row_bytes = CVPixelBufferGetBytesPerRow(image);
        const auto* base = static_cast<const std::uint8_t*>(CVPixelBufferGetBaseAddress(image));
        if (base == nullptr || width != config_.width || height != config_.height ||
            row_bytes < width * 4) {
            CVPixelBufferUnlockBaseAddress(image, kCVPixelBufferLock_ReadOnly);
            return;
        }

        std::vector<std::uint8_t> bgr(width * height * 3);
        for (std::size_t y = 0; y < height; ++y) {
            const auto* source = base + y * row_bytes;
            auto* destination = bgr.data() + y * width * 3;
            for (std::size_t x = 0; x < width; ++x) {
                destination[x * 3] = source[x * 4];
                destination[x * 3 + 1] = source[x * 4 + 1];
                destination[x * 3 + 2] = source[x * 4 + 2];
            }
        }
        CVPixelBufferUnlockBaseAddress(image, kCVPixelBufferLock_ReadOnly);

        const CMTime presentation_time = CMSampleBufferGetPresentationTimeStamp(sample_buffer);
        if (!CMTIME_IS_VALID(presentation_time)) {
            return;
        }
        if (!timestamp_origin_set_) {
            timestamp_origin_ = presentation_time;
            timestamp_origin_set_ = true;
        }
        const CMTime relative = CMTimeSubtract(presentation_time, timestamp_origin_);
        const auto timestamp_ns = static_cast<std::int64_t>(
            CMTimeGetSeconds(relative) * static_cast<double>(1'000'000'000));
        if (timestamp_ns < 0) {
            return;
        }

        try {
            ImageFrame frame(next_frame_id_.fetch_add(1), ImageFrame::Timestamp(timestamp_ns),
                             config_.width, config_.height, PixelFormat::kBgr8, std::move(bgr));
            if (sequence_validator_.Validate(frame).has_value()) {
                capture_dropped_frames_.fetch_add(1);
                return;
            }
            queue_.Push(std::move(frame));
        } catch (const std::exception&) {
            capture_dropped_frames_.fetch_add(1);
        }
    }

    void OnDroppedBuffer() noexcept { capture_dropped_frames_.fetch_add(1); }

    [[nodiscard]] const CameraCalibration& calibration() const noexcept {
        return config_.calibration;
    }
    [[nodiscard]] const MacosCaptureInfo& capture_info() const noexcept { return info_; }
    [[nodiscard]] std::uint64_t dropped_frames() const noexcept {
        return queue_.dropped_frames() + capture_dropped_frames_.load();
    }

   private:
    void ReleaseCaptureObjects() noexcept {
        if (output_ != nil) {
            [output_ setSampleBufferDelegate:nil queue:nullptr];
        }
        if (callback_queue_ != nullptr) {
            dispatch_sync(callback_queue_, ^{});
        }
        if (delegate_ != nil) {
            delegate_.context = nullptr;
        }
        delegate_ = nil;
        output_ = nil;
        input_ = nil;
        session_ = nil;
        device_ = nil;
        callback_queue_ = nullptr;
    }

    MacosCameraConfig config_;
    BoundedFrameQueue queue_;
    FrameSequenceValidator sequence_validator_;
    std::atomic<bool> cancelled_{true};
    std::atomic<std::uint64_t> next_frame_id_{0};
    std::atomic<std::uint64_t> capture_dropped_frames_{0};
    bool started_{false};
    bool timestamp_origin_set_{false};
    CMTime timestamp_origin_{kCMTimeInvalid};
    MacosCaptureInfo info_;
    AVCaptureSession* session_{nil};
    AVCaptureDevice* device_{nil};
    AVCaptureDeviceInput* input_{nil};
    AVCaptureVideoDataOutput* output_{nil};
    SlamVideoDelegate* delegate_{nil};
    dispatch_queue_t callback_queue_{nullptr};
};

void DeliverSampleBuffer(void* context, CMSampleBufferRef sample_buffer) noexcept {
    if (context != nullptr) {
        static_cast<MacosCameraSource::Impl*>(context)->OnSampleBuffer(sample_buffer);
    }
}

void DeliverDroppedBuffer(void* context) noexcept {
    if (context != nullptr) {
        static_cast<MacosCameraSource::Impl*>(context)->OnDroppedBuffer();
    }
}

CameraAuthorization GetCameraAuthorization() noexcept {
    switch ([AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo]) {
        case AVAuthorizationStatusAuthorized:
            return CameraAuthorization::kAuthorized;
        case AVAuthorizationStatusDenied:
            return CameraAuthorization::kDenied;
        case AVAuthorizationStatusRestricted:
            return CameraAuthorization::kRestricted;
        case AVAuthorizationStatusNotDetermined:
            return CameraAuthorization::kNotDetermined;
    }
}

bool RequestCameraAuthorization() {
    if (GetCameraAuthorization() == CameraAuthorization::kAuthorized) {
        return true;
    }
    dispatch_semaphore_t completed = dispatch_semaphore_create(0);
    __block bool granted = false;
    [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo
                             completionHandler:^(BOOL allowed) {
                               granted = allowed;
                               dispatch_semaphore_signal(completed);
                             }];
    dispatch_semaphore_wait(completed, DISPATCH_TIME_FOREVER);
    return granted;
}

std::vector<MacosCameraDevice> ListMacosCameraDevices() {
    std::vector<MacosCameraDevice> devices;
    AVCaptureDeviceDiscoverySession* discovery = [AVCaptureDeviceDiscoverySession
        discoverySessionWithDeviceTypes:CameraDeviceTypes()
                           mediaType:AVMediaTypeVideo
                            position:AVCaptureDevicePositionUnspecified];
    for (AVCaptureDevice* device in discovery.devices) {
        devices.push_back({ToString(device.uniqueID), ToString(device.localizedName)});
    }
    return devices;
}

MacosCameraSource::MacosCameraSource(MacosCameraConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}
MacosCameraSource::~MacosCameraSource() = default;
StartResult MacosCameraSource::Start() { return impl_->Start(); }
FrameResult MacosCameraSource::NextFrame(std::chrono::milliseconds timeout) {
    return impl_->NextFrame(timeout);
}
void MacosCameraSource::RequestStop() noexcept { impl_->RequestStop(); }
void MacosCameraSource::Stop() noexcept { impl_->Stop(); }
const CameraCalibration& MacosCameraSource::calibration() const noexcept {
    return impl_->calibration();
}
const MacosCaptureInfo& MacosCameraSource::capture_info() const noexcept {
    return impl_->capture_info();
}
std::uint64_t MacosCameraSource::dropped_frames() const noexcept {
    return impl_->dropped_frames();
}

}  // namespace slam_remote::camera
