# Smartphone camera for host SLAM

The phone captures images in a browser; the host runs ORB-SLAM3 and the existing
Rust Sender. This adds no phone app installation and does not run SLAM on the
phone itself. The pipeline is:

Phone browser → authenticated HTTPS → local SLF1 pipe → ORB-SLAM3 → Rust Sender
→ Receiver → Unity.

Use a browser providing `getUserMedia`, `requestVideoFrameCallback`, BigInt and
`AbortSignal.timeout`. The phone must trust the host's HTTPS certificate. Camera
access over a plain HTTP LAN address is unavailable; loopback HTTP is provided
only for host development. See the browser documentation for
[camera permission and secure contexts](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia)
and [video timestamps](https://developer.mozilla.org/en-US/docs/Web/API/HTMLVideoElement/requestVideoFrameCallback).
Physical phone/browser compatibility and a phone-to-Unity run must be verified
on your devices; host tests and virtual-camera tests are not that evidence.

## Prepare

1. Build `orbslam3_stream_camera_sender` as described in
   [streamed camera input](../../docs/streamed-camera-input.md) and build the Rust
   sender and receiver. Python 3.9+ is sufficient for the HTTPS host; no Python
   packages are needed for serving live camera frames.
2. Calibrate the selected phone lens at the exact transmitted resolution and
   orientation. The page's **Calibration preview** opens the camera without
   starting SLAM; **Save calibration image** downloads Gray8 PNGs from the same
   capture geometry. Collect at least 10 usable checkerboard views and transfer
   them to the host for `tools/calibrate_monocular.py`, then export the result
   with `calibration_convert`. The `--settings` destination can be prepared after
   previewing, but must exist before **Start camera**. Use the settings' `Camera.width`, `Camera.height`
   and integer `Camera.fps` below. A host webcam calibration is not valid for a
   phone. Keep zoom, lens, orientation and stabilization/cropping behavior fixed.
3. Obtain a TLS certificate with the host DNS name or LAN IP in its subject
   alternative names, plus its private key. Use a certificate issued by an
   authority your phone trusts, or explicitly install and trust your development
   CA on the test phone. A certificate warning bypass is not a reliable substitute
   for a trusted secure context. Keep private keys outside the repository.
4. Put host and phone on a network where they can reach each other; permit TCP
   8443 to the host. Guest Wi-Fi client isolation may prevent this. Permit the
   Sender's TCP 5555 from the receiving computer as well.

## Launch a camera

Start the Rust streamer on the host first:

```bash
cargo run --manifest-path sender/streamer/Cargo.toml --bin slam-mock-sender -- \
  --source live --slam-socket /private/tmp/phone-a.sock --endpoint 'tcp://*:5555'
```

Then start the camera host:

```bash
python3 sender/phone/server.py \
  --bind 0.0.0.0 --port 8443 --cert /path/to/host-cert.pem --key /path/to/host-key.pem \
  --producer /path/to/orbslam3_stream_camera_sender \
  --vocabulary /path/to/ORBvoc.txt --settings /path/to/phone-camera.yaml \
  --slam-socket /private/tmp/phone-a.sock --camera-id phone-a-rear \
  --width 640 --height 480 --fps 30
```

Open the printed pairing URL on the phone, replacing `HOST` with the certificate's
host name or IP. The fragment carries a randomly generated pairing token and is
removed from browser history after loading; it is sent only in authorization
headers, not HTTP URLs. Anyone holding the token can operate that camera host,
so share it only with its camera operator. Select the calibrated lens, then tap
**Start camera** and grant camera permission. The preview is not mirrored or
resized for transmission. Accepted-frame counts confirm host ingress, not SLAM
tracking; the producer and Receiver logs confirm poses and point-cloud updates.

On the Unity computer, launch Receiver connected to the host's TCP 5555 endpoint
and forwarding to local TCP 5556, following [Receiver setup](../../receiver/README.md).
Configure Unity's subscriber to its local Receiver endpoint as in
[Unity setup](../../unity/README.md), then enter Play Mode.

Tap **Stop** before closing the browser. Switching apps/hiding the page also
stops capture. If the network disappears, the host closes the producer pipe
after 15 seconds without an accepted frame (configurable with `--idle-timeout`).
Input backpressure is limited to 30 seconds, and shutdown allows 10 seconds for
the producer before termination. Ctrl-C or SIGTERM on the host closes the
producer too. The browser has one frame request in flight; slow processing skips
capture frames rather than building a queue. Raw 640×480 Gray8 at 30 fps needs
about 9.2 MB/s before HTTPS overhead.

There is one camera session per host launch. After Stop, producer failure or
disconnect, restart both the host and Rust streamer and use the new pairing URL.
The host generates a fresh telemetry session ID every launch; a second phone
cannot take over an active session. For another camera, start another host and
streamer with distinct HTTPS port, Unix socket, camera ID and telemetry port.
This provides input isolation; selecting multiple maps in Unity and supporting
late subscribers with complete point-cloud state remain tracked by #107.

## Limits and verification

The browser refuses changed resolution or screen orientation during capture.
It uses the presented frame's media timestamp, not network arrival time. Browser
camera processing (autofocus, stabilization and exposure) can still impair
tracking, and there is no inertial input. Monocular reconstruction has unknown
scale without a reference; protocol metre labels do not make it metric. Each
camera's map is independent; no cross-device alignment is performed here.

Run the deterministic tests (local sockets require permission):

```bash
python3 -m unittest discover -s sender/phone -v
node sender/phone/capture.test.cjs
```

The Python suite tests real HTTP and trusted HTTPS using a temporary certificate,
session ownership, exact frame forwarding, malformed input, idle cleanup and
slow/failed producer shutdown. The JavaScript suite checks the binary format and
Gray8 conversion. Neither suite proves physical camera SLAM accuracy.

On 2026-10-16, trusted HTTPS replay on Apple Silicon delivered 798 frames,
795 tracked poses, 26 point-cloud deltas and 79,469 point operations to the Rust
validator, with clean producer and session shutdown. Chromium with a virtual
camera passed calibration preview/download, capture/stop, orientation-change
stop and permission-error checks at a 390×844 viewport. Physical smartphone
capture, lens calibration and phone-to-Unity evidence are still outstanding.

Optional browser smoke tests use Playwright with Chromium's fake camera:

```bash
npm install --prefix /private/tmp/slam-phone-browser-test playwright
/private/tmp/slam-phone-browser-test/node_modules/.bin/playwright install chromium
NODE_PATH=/private/tmp/slam-phone-browser-test/node_modules node sender/phone/browser.test.cjs
```

`replay_test.py` exercises the complete HTTPS-to-SLAM-to-Rust path using the TUM
fixture from `docs/slam-host.md`. Run with a Python environment containing OpenCV:

```bash
/private/tmp/slam-remote-viewer-orbslam3/env/bin/python sender/phone/replay_test.py \
  --producer /private/tmp/slam-goal-orb-adapter/orbslam3_stream_camera_sender \
  --orb-root /private/tmp/slam-remote-viewer-orbslam3/src/ORB_SLAM3 \
  --sequence /private/tmp/slam-remote-viewer-orbslam3/datasets/rgbd_dataset_freiburg1_xyz \
  --boundary-dump sender/streamer/target/debug/slam_boundary_dump
```
