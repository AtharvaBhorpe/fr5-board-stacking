import { chromium, expect } from "@playwright/test";
import { resolve } from "node:path";
import { writeFile } from "node:fs/promises";
const browser = await chromium.launch({
  executablePath: "/usr/bin/google-chrome",
  headless: true,
  args: [
    "--no-sandbox",
    "--use-gl=angle",
    "--use-angle=swiftshader",
    "--enable-unsafe-swiftshader",
    "--use-fake-ui-for-media-stream",
    "--use-fake-device-for-media-stream",
    `--use-file-for-fake-audio-capture=${resolve("../conversation/tests/pick_two_varb.wav")}`,
  ],
});
const page = await browser.newPage({ permissions: ["microphone"] });
await page.addInitScript(() => {
  Object.defineProperty(navigator, "audioSession", {
    value: { type: "auto" }, configurable: true,
  });
});
let captures = [];
await page.route("**/api/state", async (route) => {
  const response = await route.fetch();
  const data = await response.json();
  data.model.asr_ready = true;
  await route.fulfill({ response, json: data });
});
await page.route("**/api/transcribe", async (route) => {
  const data = route.request().postDataJSON();
  const b = Buffer.from(data.wav, "base64");
  captures.push({
    bytes: b.length,
    rate: b.readUInt32LE(24),
    channels: b.readUInt16LE(22),
    bits: b.readUInt16LE(34),
    seconds: (b.length - 44) / 32000,
  });
  await route.fulfill({
    json: {
      text: "Pick two variant B boards.",
      normalized: "Pick two variant B boards.",
    },
  });
});
await page.goto("http://localhost:8088");
await page.locator("#voice-language").selectOption("de");
await page.reload();
await expect(page.locator("#voice-language")).toHaveValue("de");
await expect(page.locator("#talk")).toBeEnabled();
await page.locator("#talk").scrollIntoViewIfNeeded();
let box = await page.locator("#talk").boundingBox();
await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
await page.mouse.down();
await expect(page.locator("#talk")).toHaveClass(/recording/);
expect(await page.evaluate(() => navigator.audioSession.type)).toBe("play-and-record");
await page.waitForTimeout(1600);
await page.mouse.up();
await expect(page.locator("#voice-status")).toContainText(
  "Heard: Pick two variant B boards.",
);
expect(await page.evaluate(() => navigator.audioSession.type)).toBe("playback");
if (
  captures.length !== 1 ||
  captures[0].rate !== 16000 ||
  captures[0].channels !== 1 ||
  captures[0].bits !== 16 ||
  captures[0].seconds < 1
)
  throw new Error("Invalid browser WAV capture");
await page.mouse.down();
await expect(page.locator("#talk")).toHaveClass(/recording/);
await page.waitForTimeout(300);
await page.locator("#talk").dispatchEvent("pointercancel");
await page.mouse.up();
await page.waitForTimeout(500);
if (captures.length !== 1) throw new Error("Cancelled capture was submitted");
expect(await page.evaluate(() => navigator.audioSession.type)).toBe("playback");
await page.evaluate(() => {
  navigator.mediaDevices.getUserMedia = async () => {
    throw new DOMException("Permission denied", "NotAllowedError");
  };
});
await page.mouse.down();
await expect(page.locator("#voice-status")).toContainText("Microphone unavailable");
await page.mouse.up();
expect(await page.evaluate(() => navigator.audioSession.type)).toBe("playback");
await writeFile(
  "test-results/microphone.json",
  JSON.stringify(
    {
      passed: true,
      synthetic_device: true,
      asr_stub: true,
      robot_commands_sent: false,
      captures,
      cancel_discards: true,
    },
    null,
    2,
  ),
);
console.log({ captures, cancel_discards: true, asr_stub: true });
await browser.close();
