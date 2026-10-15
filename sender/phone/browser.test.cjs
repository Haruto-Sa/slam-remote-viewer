// Optional: npm install --prefix /private/tmp/slam-phone-browser-test playwright
// NODE_PATH=/private/tmp/slam-phone-browser-test/node_modules node sender/phone/browser.test.cjs
const { chromium } = require("playwright");
const { spawn } = require("node:child_process");
const { once } = require("node:events");
const assert = require("node:assert/strict");
const path = require("node:path");

async function host() {
  const child = spawn("python3", ["-u", path.join(__dirname, "server.py"),
    "--port", "0", "--producer", "python3", "--vocabulary=-c",
    "--settings=import sys; sys.stdin.buffer.read()", "--slam-socket", "/unused-test-socket",
    "--camera-id", "browser-test", "--width", "640", "--height", "480"],
    { stdio: ["ignore", "pipe", "inherit"] });
  let output = "";
  try {
    const url = await new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error("host startup timeout")), 10000);
      child.on("error", error => { clearTimeout(timer); reject(error); });
      child.on("exit", code => { clearTimeout(timer); reject(new Error(`host exited ${code}`)); });
      child.stdout.on("data", chunk => {
        output += chunk;
        const match = output.match(/Open (http:\/\/HOST:\d+\/#token=\S+)/);
        if (match) { clearTimeout(timer); resolve(match[1].replace("HOST", "127.0.0.1")); }
      });
    });
    return { child, url };
  } catch (error) { child.kill("SIGTERM"); throw error; }
}

(async () => {
  const browser = await chromium.launch({ headless: true,
    args: ["--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"] });
  try {
    for (const scenario of ["stop", "orientation", "permission-error"]) {
      const { child, url } = await host();
      const context = await browser.newContext({ viewport: { width: 390, height: 844 } });
      try {
        const page = await context.newPage();
        const errors = [];
        page.on("pageerror", error => errors.push(error.message));
        await page.goto(url);
        assert.equal(new URL(page.url()).hash, "");
        if (scenario === "stop") {
          await page.click("#calibrate");
          await page.waitForFunction(() => !document.querySelector("#save").disabled);
          const downloaded = page.waitForEvent("download");
          await page.click("#save");
          assert.match((await downloaded).suggestedFilename(), /^calibration-640x480-\d+\.png$/);
          await page.click("#stop");
          assert.equal(await page.locator("#start").isEnabled(), true);
        }
        if (scenario === "permission-error") {
          await page.evaluate(() => {
            navigator.mediaDevices.getUserMedia = async () => { throw new Error("Permission denied by test"); };
          });
          await page.click("#start");
          await page.waitForFunction(() => document.querySelector("#status").textContent.includes("Permission denied"));
          assert.equal(await page.locator("#start").isEnabled(), true);
        } else {
          await page.click("#start");
          await page.waitForFunction(() => /Sending camera frames: [5-9]/.test(document.querySelector("#status").textContent));
          if (scenario === "stop") await page.click("#stop");
          else await page.evaluate(() => Object.defineProperty(screen.orientation, "angle", { value: 90 }));
          await page.waitForFunction(() => document.querySelector("#status").textContent.includes("Restart host and streamer"));
          assert.equal(await page.locator("#stop").isDisabled(), true);
          assert.equal(await page.evaluate(() => document.querySelector("#preview").srcObject), null);
          if (scenario === "orientation") assert.match(await page.locator("#status").textContent(), /orientation/);
          if (scenario === "stop") await page.screenshot({ path: "/private/tmp/slam-phone-browser.png", fullPage: true });
        }
        assert.deepEqual(errors, []);
        console.log(`browser scenario passed: ${scenario}`);
      } finally {
        await context.close();
        const exited = once(child, "exit");
        child.kill("SIGTERM");
        await exited;
      }
    }
  } finally { await browser.close(); }
})().catch(error => { console.error(error); process.exitCode = 1; });
