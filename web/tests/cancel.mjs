import { chromium, expect } from "@playwright/test";
import { writeFile } from "node:fs/promises";
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
const page = await browser.newPage();
await page.goto("http://localhost:8088");
await page.getByText("Live joint positions", { exact: true }).waitFor();
await page.locator('[data-reset="mixed"]').click();
await expect(page.locator('[data-variant="VARa"]')).toBeEnabled();
await page.locator('[data-variant="VARa"]').click();
await expect(page.locator("#controller")).toHaveText("running", {
  timeout: 30000,
});
await page.locator("#stop").click();
await expect(page.locator("#controller")).toHaveText("stopped", {
  timeout: 15000,
});
const token = await page.evaluate(() => localStorage.getItem("workcell-token"));
const state = await (
  await page.request.get("http://localhost:8088/api/state", {
    headers: { "X-Workcell-Token": token },
  })
).json();
await writeFile(
  "test-results/cancel.json",
  JSON.stringify(
    { passed: true, panel: state.panel, events: state.events },
    null,
    2,
  ),
);
await page.locator('[data-reset="two_b"]').click();
await browser.close();
console.log("Variant button and software cancellation passed");
