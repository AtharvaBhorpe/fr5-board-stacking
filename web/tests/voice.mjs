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
    "--autoplay-policy=no-user-gesture-required",
    "--use-fake-ui-for-media-stream",
    "--use-fake-device-for-media-stream",
    `--use-file-for-fake-audio-capture=${resolve("../conversation/results/voice-batch/0-normal.wav")}%noloop`,
  ],
});
const context = await browser.newContext({
  permissions: ["microphone"],
  viewport: { width: 1440, height: 1000 },
});
const page = await context.newPage();
const errors = [];
page.on("pageerror", (e) => errors.push(e.message));
await page.goto("http://localhost:8088");
await page.getByText("Live joint positions", { exact: true }).waitFor();
await page.locator('[data-reset="two_b"]').click();
await page.waitForTimeout(1800);
await page.locator("#sound").click();
const synthesis = [];
page.on("response", (r) => {
  if (r.url().endsWith("/api/speech")) synthesis.push(r.status());
});
if (process.env.TYPED === "1") {
  await page.locator("#chat-input").fill("pick two variant B boards");
  await page.locator("#send").click();
} else {
  const talk = page.locator("#talk");
  await talk.scrollIntoViewIfNeeded();
  const box = await talk.boundingBox();
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await page.mouse.down();
  await expect(page.locator("#talk")).toHaveClass(/recording/);
  await page.waitForTimeout(4200);
  await page.mouse.up();
  await expect(page.locator("#voice-status")).toContainText(
    "Pick two variant B boards",
    { timeout: 60000 },
  );
}
await expect(page.locator("#batch")).toHaveText("2 / 2 · completed", {
  timeout: 120000,
});
await expect(page.locator("#messages")).toContainText(
  "Second board placed. All 2 boards are done.",
);
await page.waitForTimeout(5000);
const token = await page.evaluate(() => localStorage.getItem("workcell-token"));
const state = await (
  await context.request.get("http://localhost:8088/api/state", {
    headers: { "X-Workcell-Token": token },
  })
).json();
await writeFile(
  process.env.TYPED === "1"
    ? "test-results/typed.json"
    : "test-results/voice.json",
  JSON.stringify(
    {
      passed: true,
      synthetic_microphone: process.env.TYPED !== "1",
      physical_microphone_verified: false,
      synthesis,
      errors,
      batch: state.batch,
      events: state.events,
    },
    null,
    2,
  ),
);
await page.screenshot({ path: "test-results/completed.png", fullPage: true });
await page.locator('[data-reset="two_b"]').click();
console.log({ batch: state.batch, synthesis, errors });
await browser.close();
if (errors.length || !synthesis.length || synthesis.some((x) => x !== 200))
  process.exit(1);
