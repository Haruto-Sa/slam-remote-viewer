"use strict";

// Pure encoder is also exercised by Node tests. No image scaling or mirroring.
function encodeFrame(width, height, frameId, timestampNs, rgba) {
  if (![width, height, frameId, timestampNs].every(Number.isSafeInteger) ||
      width <= 0 || height <= 0 || width > 1920 || height > 1920 ||
      width * height > 2073600 || frameId < 0 || timestampNs < 0 ||
      rgba.length !== width * height * 4) throw new Error("Invalid capture frame");
  const buffer = new ArrayBuffer(28 + width * height);
  const view = new DataView(buffer);
  view.setUint32(0, 0x534c4631); // SLF1, big endian
  view.setUint32(4, width);
  view.setUint32(8, height);
  view.setBigUint64(12, BigInt(frameId));
  view.setBigUint64(20, BigInt(timestampNs));
  const pixels = new Uint8Array(buffer, 28);
  for (let p = 0; p < pixels.length; ++p) {
    pixels[p] = Math.round(.299 * rgba[p * 4] + .587 * rgba[p * 4 + 1] + .114 * rgba[p * 4 + 2]);
  }
  return buffer;
}

if (typeof module !== "undefined") module.exports = { encodeFrame };

if (typeof document !== "undefined") {
  const byId = id => document.getElementById(id);
  const video = byId("preview"), status = byId("status"), token = byId("token");
  const fragment = new URLSearchParams(location.hash.slice(1));
  token.value = fragment.get("token") || "";
  if (location.hash) history.replaceState(null, "", location.pathname);
  let stream = null, lease = null, running = false, starting = false, calibrating = false, frameRequest = null;
  let callbackId = null, captureToken = "";
  let calibrationGeometry = null;
  const currentOrientation = () => screen.orientation ? screen.orientation.angle : window.orientation;

  async function request(path, body, signal) {
    const headers = { Authorization: "Bearer " + (captureToken || token.value.trim()) };
    if (lease) headers["X-Slam-Session"] = lease;
    if (body) headers["Content-Type"] = "application/octet-stream";
    const response = await fetch(path, {
      method: path === "/status" ? "GET" : "POST", headers,
      body: body || (path === "/status" ? undefined : ""), signal
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || `Host error ${response.status}`);
    return result;
  }

  function releaseCamera() {
    if (callbackId !== null) video.cancelVideoFrameCallback(callbackId);
    callbackId = null;
    if (stream) stream.getTracks().forEach(track => track.stop());
    stream = null;
    video.srcObject = null;
  }

  async function stop(message = "Camera stopped.") {
    running = false;
    calibrating = false;
    if (frameRequest) frameRequest.abort();
    releaseCamera();
    byId("stop").disabled = true;
    byId("save").disabled = true;
    status.textContent = message;
    if (lease) {
      try {
        const result = await request("/stop", null, AbortSignal.timeout(45000));
        status.textContent = `${message}${result.state === "failed" ? " " + result.detail + "." : ""} Restart host and streamer for a new session.`;
      } catch (error) {
        status.textContent = `${message} ${error.message}. The host also stops after idle timeout.`;
      }
      lease = null;
    }
    if (!token.disabled) {
      byId("start").disabled = byId("calibrate").disabled = byId("discover").disabled = byId("camera").disabled = false;
    }
  }

  async function openCamera(config) {
    if (!isSecureContext || !navigator.mediaDevices) throw new Error("Open this page over trusted HTTPS to use the camera");
    releaseCamera();
    const selected = byId("camera").value;
    stream = await navigator.mediaDevices.getUserMedia({ audio: false, video: {
      width: { exact: config.width }, height: { exact: config.height },
      frameRate: { ideal: config.fps, max: config.fps }, resizeMode: "none",
      ...(selected ? { deviceId: { exact: selected } } : { facingMode: { ideal: "environment" } })
    } });
    video.srcObject = stream;
    await video.play();
    if (document.hidden) throw new Error("Keep the camera page visible while starting");
    if (video.videoWidth !== config.width || video.videoHeight !== config.height) {
      throw new Error("Camera resolution does not match the host calibration");
    }
    const track = stream.getVideoTracks()[0];
    byId("details").textContent = `${track.label}: ${config.width} × ${config.height}, up to ${config.fps} fps. Use calibration for this lens and orientation.`;
    return track;
  }

  byId("calibrate").onclick = async () => {
    if (starting || running) return;
    starting = true;
    byId("start").disabled = byId("calibrate").disabled = byId("discover").disabled = byId("camera").disabled = true;
    try {
      captureToken = token.value.trim();
      const config = await request("/status", null, AbortSignal.timeout(10000));
      if (config.state !== "ready") throw new Error("Restart the host before calibration");
      const track = await openCamera(config);
      calibrating = true;
      calibrationGeometry = [video.videoWidth, video.videoHeight, currentOrientation()];
      track.onended = () => { if (calibrating) void stop("Camera disconnected."); };
      byId("save").disabled = byId("stop").disabled = false;
      status.textContent = "Move the checkerboard across the view and save several images. Keep this lens and phone orientation fixed. Stop preview before starting SLAM.";
    } catch (error) { await stop(`Preview failed: ${error.message}.`); }
    finally { starting = false; }
  };

  byId("save").onclick = () => {
    if (!calibrating) return;
    if (video.videoWidth !== calibrationGeometry[0] || video.videoHeight !== calibrationGeometry[1] ||
        currentOrientation() !== calibrationGeometry[2]) {
      void stop("Calibration orientation or dimensions changed. Start a new consistent image set.");
      return;
    }
    const canvas = document.createElement("canvas");
    canvas.width = video.videoWidth; canvas.height = video.videoHeight;
    const context = canvas.getContext("2d", { willReadFrequently: true });
    context.drawImage(video, 0, 0);
    const rgba = context.getImageData(0, 0, canvas.width, canvas.height);
    const gray = new Uint8Array(encodeFrame(canvas.width, canvas.height, 0, 0, rgba.data), 28);
    for (let i = 0; i < gray.length; ++i) {
      rgba.data[i * 4] = rgba.data[i * 4 + 1] = rgba.data[i * 4 + 2] = gray[i];
    }
    context.putImageData(rgba, 0, 0);
    canvas.toBlob(blob => {
      if (!blob) { status.textContent = "Could not save calibration image"; return; }
      const url = URL.createObjectURL(blob);
      const link = document.createElement("a");
      link.href = url;
      link.download = `calibration-${canvas.width}x${canvas.height}-${Date.now()}.png`;
      link.click();
      setTimeout(() => URL.revokeObjectURL(url), 60000);
    }, "image/png");
  };

  byId("discover").onclick = async () => {
    if (starting || running || calibrating) return;
    starting = true;
    byId("discover").disabled = byId("start").disabled = byId("calibrate").disabled = true;
    let probe;
    try {
      probe = await navigator.mediaDevices.getUserMedia({ video: true, audio: false });
      const devices = await navigator.mediaDevices.enumerateDevices();
      byId("camera").replaceChildren(new Option("Rear camera (default)", ""));
      for (const device of devices.filter(d => d.kind === "videoinput")) {
        byId("camera").add(new Option(device.label || "Camera", device.deviceId));
      }
      status.textContent = "Select the lens used for your calibration, then start.";
    } catch (error) { status.textContent = `Camera access failed: ${error.message}`; }
    finally {
      if (probe) probe.getTracks().forEach(track => track.stop());
      byId("discover").disabled = byId("start").disabled = byId("calibrate").disabled = false;
      starting = false;
    }
  };

  byId("start").onclick = async () => {
    if (starting || running) return;
    starting = true;
    byId("start").disabled = byId("calibrate").disabled = byId("discover").disabled = byId("camera").disabled = true;
    try {
      if (!isSecureContext || !navigator.mediaDevices) throw new Error("Open this page over trusted HTTPS to use the camera");
      if (!video.requestVideoFrameCallback || !AbortSignal.timeout) throw new Error("Use a current browser with video frame callbacks and request timeouts");
      captureToken = token.value.trim();
      if (!captureToken) throw new Error("Enter the host pairing token");
      const config = await request("/status", null, AbortSignal.timeout(10000));
      if (config.state !== "ready") throw new Error(config.detail + "; restart host and streamer");
      const track = await openCamera(config);
      lease = (await request("/start", null, AbortSignal.timeout(10000))).session;
      token.disabled = true;
      running = true;
      byId("stop").disabled = false;
      track.onended = () => { if (running) void stop("Camera disconnected."); };
      const canvas = document.createElement("canvas");
      canvas.width = config.width; canvas.height = config.height;
      const context = canvas.getContext("2d", { willReadFrequently: true });
      let lastTimestamp = -1, sent = 0;
      const orientation = currentOrientation();
      async function capture(_now, metadata) {
        if (!running) return;
        try {
          if (video.videoWidth !== config.width || video.videoHeight !== config.height ||
              orientation !== currentOrientation()) {
            throw new Error("Camera orientation or dimensions changed; recalibrate before restarting");
          }
          const timestamp = Math.round(metadata.mediaTime * 1e9);
          if (timestamp > lastTimestamp && (lastTimestamp < 0 || timestamp - lastTimestamp >= 1e9 / config.fps - 1000)) {
            context.drawImage(video, 0, 0);
            const rgba = context.getImageData(0, 0, config.width, config.height).data;
            const bytes = encodeFrame(config.width, config.height, metadata.presentedFrames, timestamp, rgba);
            frameRequest = new AbortController();
            const timer = setTimeout(() => frameRequest?.abort(), 45000);
            try { await request("/frame", bytes, frameRequest.signal); }
            finally { clearTimeout(timer); frameRequest = null; }
            lastTimestamp = timestamp;
            if (running) status.textContent = `Sending camera frames: ${++sent} accepted by host.`;
          }
          if (running) callbackId = video.requestVideoFrameCallback(capture);
        } catch (error) { if (running) await stop(`Capture failed: ${error.message}.`); }
      }
      callbackId = video.requestVideoFrameCallback(capture);
    } catch (error) {
      await stop(`Cannot start: ${error.message}.`);
      if (!token.disabled) {
        byId("start").disabled = byId("calibrate").disabled = byId("discover").disabled = byId("camera").disabled = false;
      }
    } finally { starting = false; }
  };
  byId("stop").onclick = () => { void stop(); };
  document.addEventListener("visibilitychange", () => {
    if (document.hidden && (running || calibrating)) void stop("Capture stopped because the page was hidden.");
  });
  window.addEventListener("pagehide", () => {
    running = false;
    releaseCamera(); // Server idle timeout closes the producer if the request is interrupted.
  });
}
