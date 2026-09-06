#import <AppKit/AppKit.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "slam_remote/launcher/macos_sender_control.hpp"

namespace {

using slam_remote::launcher::BuildLauncherArguments;
using slam_remote::launcher::ControlStateName;
using slam_remote::launcher::SenderControlModel;
using slam_remote::launcher::SenderLaunchConfig;

NSString* const kFieldKeys[] = {@"launcher",   @"streamer", @"producer", @"vocabulary",
                                @"settings",   @"device",   @"width",    @"height",
                                @"fps",        @"socket",   @"endpoint", @"session",
                                @"camera",     @"period"};
NSString* const kFieldLabels[] = {@"Launcher",   @"Rust streamer", @"C++ producer", @"Vocabulary",
                                  @"Settings",   @"Camera device", @"Width",        @"Height",
                                  @"FPS",        @"SLAM socket",   @"PUB endpoint", @"Session",
                                  @"Camera ID",  @"Point period"};
NSString* const kDefaults[] = {@"sender/streamer/target/debug/macos_live_sender",
                               @"sender/streamer/target/debug/slam-mock-sender",
                               @"/private/tmp/slam-pose-adapter/orbslam3_macos_camera_sender",
                               @"", @"", @"", @"640", @"480", @"30",
                               @"/private/tmp/slam-live.sock", @"tcp://*:5555",
                               @"live-session", @"mac-camera", @"30"};
constexpr std::size_t kFieldCount = sizeof(kFieldKeys) / sizeof(kFieldKeys[0]);

std::string Utf8(NSString* value) { return value.UTF8String != nullptr ? value.UTF8String : ""; }

}  // namespace

@interface SenderControlDelegate : NSObject <NSApplicationDelegate, NSWindowDelegate>
@end

@implementation SenderControlDelegate {
    NSWindow* _window;
    NSMutableDictionary<NSString*, NSTextField*>* _fields;
    NSTextField* _status;
    NSButton* _start;
    NSButton* _stop;
    NSTask* _task;
    SenderControlModel _model;
    BOOL _closeWhenStopped;
}

- (void)applicationDidFinishLaunching:(NSNotification*)notification {
    (void)notification;
    _fields = [NSMutableDictionary dictionary];
    _window = [[NSWindow alloc]
        initWithContentRect:NSMakeRect(0, 0, 820, 680)
                  styleMask:(NSWindowStyleMaskTitled | NSWindowStyleMaskClosable |
                             NSWindowStyleMaskMiniaturizable)
                    backing:NSBackingStoreBuffered
                      defer:NO];
    _window.title = @"SLAM Live Sender";
    _window.delegate = self;
    NSView* content = _window.contentView;
    NSUserDefaults* defaults = NSUserDefaults.standardUserDefaults;
    CGFloat y = 630;
    for (std::size_t index = 0; index < kFieldCount; ++index) {
        NSTextField* label = [NSTextField labelWithString:kFieldLabels[index]];
        label.frame = NSMakeRect(20, y, 125, 24);
        NSTextField* field = [[NSTextField alloc] initWithFrame:NSMakeRect(150, y, 645, 24)];
        NSString* saved = [defaults stringForKey:kFieldKeys[index]];
        field.stringValue = saved != nil ? saved : kDefaults[index];
        [content addSubview:label];
        [content addSubview:field];
        _fields[kFieldKeys[index]] = field;
        y -= 40;
    }

    _start = [NSButton buttonWithTitle:@"Start" target:self action:@selector(start:)];
    _start.frame = NSMakeRect(20, 25, 110, 34);
    _start.bezelStyle = NSBezelStyleRounded;
    [content addSubview:_start];
    _stop = [NSButton buttonWithTitle:@"Stop" target:self action:@selector(stop:)];
    _stop.frame = NSMakeRect(140, 25, 110, 34);
    _stop.bezelStyle = NSBezelStyleRounded;
    [content addSubview:_stop];
    _status = [NSTextField labelWithString:@"Idle"];
    _status.frame = NSMakeRect(270, 31, 525, 24);
    [content addSubview:_status];
    [self refreshControls];
    [_window center];
    [_window makeKeyAndOrderFront:nil];
    [_window makeFirstResponder:_fields[@"vocabulary"]];
    [NSApp activateIgnoringOtherApps:YES];
}

- (NSString*)value:(NSString*)key { return _fields[key].stringValue; }

- (BOOL)readPositive:(NSString*)key output:(std::uint32_t*)output {
    NSString* value = [self value:key];
    NSScanner* scanner = [NSScanner scannerWithString:value];
    unsigned long long parsed = 0;
    if (![scanner scanUnsignedLongLong:&parsed] || !scanner.isAtEnd || parsed == 0 ||
        parsed > std::numeric_limits<std::uint32_t>::max()) {
        _status.stringValue = [NSString stringWithFormat:@"%@ must be a positive uint32", key];
        return NO;
    }
    *output = static_cast<std::uint32_t>(parsed);
    return YES;
}

- (BOOL)readConfig:(SenderLaunchConfig*)config {
    config->launcher_path = Utf8([self value:@"launcher"]);
    config->streamer_path = Utf8([self value:@"streamer"]);
    config->producer_path = Utf8([self value:@"producer"]);
    config->vocabulary_path = Utf8([self value:@"vocabulary"]);
    config->settings_path = Utf8([self value:@"settings"]);
    config->device_id = Utf8([self value:@"device"]);
    config->socket_path = Utf8([self value:@"socket"]);
    config->endpoint = Utf8([self value:@"endpoint"]);
    config->session = Utf8([self value:@"session"]);
    config->camera_id = Utf8([self value:@"camera"]);
    return [self readPositive:@"width" output:&config->width] &&
           [self readPositive:@"height" output:&config->height] &&
           [self readPositive:@"fps" output:&config->fps] &&
           [self readPositive:@"period" output:&config->pointcloud_period];
}

- (void)saveFields {
    NSUserDefaults* defaults = NSUserDefaults.standardUserDefaults;
    for (std::size_t index = 0; index < kFieldCount; ++index) {
        [defaults setObject:[self value:kFieldKeys[index]] forKey:kFieldKeys[index]];
    }
}

- (void)start:(id)sender {
    (void)sender;
    SenderLaunchConfig config;
    if (![self readConfig:&config]) return;
    std::string error;
    if (!_model.RequestStart(config, error)) {
        [self refreshControls];
        _status.stringValue = [NSString stringWithUTF8String:error.c_str()];
        return;
    }
    [self saveFields];
    _task = [[NSTask alloc] init];
    _task.executableURL = [NSURL fileURLWithPath:[self value:@"launcher"]];
    NSMutableArray<NSString*>* arguments = [NSMutableArray array];
    for (const auto& argument : BuildLauncherArguments(config)) {
        [arguments addObject:[NSString stringWithUTF8String:argument.c_str()]];
    }
    _task.arguments = arguments;
    __weak SenderControlDelegate* weakSelf = self;
    _task.terminationHandler = ^(NSTask* task) {
      dispatch_async(dispatch_get_main_queue(), ^{
        SenderControlDelegate* strongSelf = weakSelf;
        if (strongSelf == nil) return;
        [strongSelf processExited:task.terminationStatus];
      });
    };
    NSError* launchError = nil;
    if (![_task launchAndReturnError:&launchError]) {
        _model.LaunchFailed();
        _task = nil;
        [self refreshControls];
        _status.stringValue =
            [NSString stringWithFormat:@"Launch failed: %@", launchError.localizedDescription];
        return;
    }
    _model.MarkRunning();
    [self refreshControls];
}

- (void)stop:(id)sender {
    (void)sender;
    if (!_model.CanStop()) return;
    _model.RequestStop();
    if (_task.running) [_task interrupt];
    [self refreshControls];
}

- (void)processExited:(int)status {
    _model.ProcessExited(status);
    _task = nil;
    [self refreshControls];
    _status.stringValue = status == 0
                              ? @"Stopped cleanly (exit 0)"
                              : [NSString stringWithFormat:@"Failed (exit %d)", status];
    if (_closeWhenStopped) [_window close];
}

- (void)refreshControls {
    _start.enabled = _model.CanStart();
    _stop.enabled = _model.CanStop();
    _status.stringValue = [NSString stringWithFormat:@"%s", ControlStateName(_model.state())];
}

- (BOOL)windowShouldClose:(NSWindow*)sender {
    (void)sender;
    if (_model.CanStop()) {
        _closeWhenStopped = YES;
        [self stop:nil];
        return NO;
    }
    return YES;
}

- (void)windowWillClose:(NSNotification*)notification {
    (void)notification;
    [NSApp terminate:nil];
}

@end

int main() {
    @autoreleasepool {
        NSApplication* application = NSApplication.sharedApplication;
        [application setActivationPolicy:NSApplicationActivationPolicyRegular];
        NSMenu* mainMenu = [[NSMenu alloc] initWithTitle:@""];
        NSMenuItem* applicationItem = [[NSMenuItem alloc] initWithTitle:@"" action:nil keyEquivalent:@""];
        [mainMenu addItem:applicationItem];
        NSMenu* applicationMenu = [[NSMenu alloc] initWithTitle:@"SLAM Live Sender"];
        NSMenuItem* quitItem = [[NSMenuItem alloc] initWithTitle:@"Quit SLAM Live Sender"
                                                         action:@selector(terminate:)
                                                  keyEquivalent:@"q"];
        [applicationMenu addItem:quitItem];
        applicationItem.submenu = applicationMenu;

        NSMenuItem* editMenuItem =
            [[NSMenuItem alloc] initWithTitle:@"Edit" action:nil keyEquivalent:@""];
        [mainMenu addItem:editMenuItem];
        NSMenu* editMenu = [[NSMenu alloc] initWithTitle:@"Edit"];
        [editMenu addItemWithTitle:@"Undo" action:@selector(undo:) keyEquivalent:@"z"];
        [editMenu addItemWithTitle:@"Redo" action:@selector(redo:) keyEquivalent:@"Z"];
        [editMenu addItem:[NSMenuItem separatorItem]];
        [editMenu addItemWithTitle:@"Cut" action:@selector(cut:) keyEquivalent:@"x"];
        [editMenu addItemWithTitle:@"Copy" action:@selector(copy:) keyEquivalent:@"c"];
        [editMenu addItemWithTitle:@"Paste" action:@selector(paste:) keyEquivalent:@"v"];
        [editMenu addItemWithTitle:@"Select All"
                            action:@selector(selectAll:)
                     keyEquivalent:@"a"];
        editMenuItem.submenu = editMenu;
        application.mainMenu = mainMenu;
        SenderControlDelegate* delegate = [[SenderControlDelegate alloc] init];
        application.delegate = delegate;
        [application run];
    }
    return EXIT_SUCCESS;
}
