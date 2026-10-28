import { chromium } from "@playwright/test";
import { mkdir, writeFile } from "node:fs/promises";
const browser = await chromium.launch({
  executablePath: "/usr/bin/google-chrome",
  headless: true,
  args: [
    "--no-sandbox",
    "--use-gl=angle",
    "--use-angle=swiftshader",
    "--enable-unsafe-swiftshader",
  ],
});
const errors = [];
const page = await browser.newPage({ viewport: { width: 1512, height: 982 } });
page.on("pageerror", (e) => errors.push(e.message));
page.on("console", (m) => {
  if (m.type() === "error") errors.push(m.text());
});
await page.goto("http://localhost:8088");
await page
  .getByText("Live joint positions", { exact: true })
  .waitFor({ timeout: 30000 });
await page.waitForFunction(
  () =>
    window.workcellRobot && Object.keys(window.workcellRobot.joints).length > 5,
);
await page.waitForTimeout(2500);
await mkdir("test-results", { recursive: true });
await page.screenshot({ path: "test-results/desktop.png", fullPage: true });
const desktop = await page.evaluate(() => ({
  overflow: document.documentElement.scrollWidth > innerWidth,
  joints: Object.keys(window.workcellRobot.joints).length,
  meshes: (() => {
    let n = 0;
    window.workcellRobot.traverse((o) => {
      if (o.isMesh) n++;
    });
    return n;
  })(),
  camera: document.querySelector("#overhead").naturalWidth,
  serviceWorker: !!navigator.serviceWorker.controller,
}));
await page.setViewportSize({ width: 390, height: 844 });
await page.waitForTimeout(500);
await page.screenshot({ path: "test-results/mobile.png", fullPage: true });
const mobile = await page.evaluate(() => ({
  overflow: document.documentElement.scrollWidth > innerWidth,
}));
await writeFile(
  "test-results/browser.json",
  JSON.stringify({ desktop, mobile, errors }, null, 2),
);
console.log({ desktop, mobile, errors });
await browser.close();
if (
  errors.length ||
  desktop.overflow ||
  mobile.overflow ||
  !desktop.meshes ||
  !desktop.camera
)
  process.exit(1);
