import { chromium, expect } from "@playwright/test";
import { mkdir, writeFile } from "node:fs/promises";

// Read live visuals, but intercept EVERY API write: these tests cannot command a robot.
const browser = await chromium.launch({
  executablePath: "/usr/bin/google-chrome",
  headless: true,
  args: ["--no-sandbox", "--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
});
await mkdir("test-results/responsive", { recursive: true });
const results = [];
try {
  for (const [width, height] of [[320, 568], [390, 844], [844, 390], [768, 1024], [1024, 768], [1440, 1000]]) {
    const context = await browser.newContext({ viewport: { width, height }, hasTouch: true, serviceWorkers: "block" });
    const page = await context.newPage();
    const writes = [], errors = [];
    let offline = false;
    page.on("pageerror", (error) => errors.push(error.message));
    await page.route("**/api/**", async (route) => {
      if (route.request().method() === "POST") {
        writes.push({ path: new URL(route.request().url()).pathname, body: route.request().postDataJSON() });
        await route.fulfill({ json: {} });
      } else if (route.request().url().endsWith("/api/state")) {
        if (offline) return route.fulfill({ status: 503, json: { error: "Test disconnection" } });
        const response = await route.fetch();
        const data = await response.json();
        data.panel.phase = "paused";
        data.robot_fresh = true;
        data.model.asr_ready = true;
        await route.fulfill({ response, json: data });
      } else await route.continue();
    });
    await page.goto("http://localhost:8088");
    await expect(page.locator("#paused-state")).toBeVisible();
    await page.waitForFunction(() => window.workcellRobot);
    await expect(page.locator("#resume")).toBeEnabled();
    await page.locator("#resume").click();
    expect(writes.some((r) => r.path === "/api/chat" && r.body.text === "resume")).toBe(true);
    await page.evaluate(() => scrollTo(0, 0));
    await page.screenshot({ path: `test-results/responsive/${width}-workspace.png`, fullPage: true });
    if (width <= 740 || height <= 500) {
      const talk = await page.locator("#talk").boundingBox();
      expect(talk.y + talk.height).toBeLessThanOrEqual(height);
      expect(talk.height).toBeGreaterThanOrEqual(44);
      await page.getByRole("button", { name: "Cameras", exact: true }).tap();
      await expect(page.locator(".workspace")).toBeHidden();
      await expect(page.locator(".vision-column")).toBeVisible();
      await expect(page.locator("#talk")).toBeVisible();
      await page.screenshot({ path: `test-results/responsive/${width}-cameras.png`, fullPage: true });
      await page.getByRole("button", { name: "Chat", exact: true }).tap();
      await expect(page.locator("#chat-form")).toBeVisible();
      await expect(page.locator(".vision-column")).toBeHidden();
      await page.screenshot({ path: `test-results/responsive/${width}-chat.png`, fullPage: true });
    }
    if (width > 740 && width < 1280 && height > 500) {
      await page.getByRole("button", { name: "Cameras", exact: true }).tap();
      await expect(page.locator(".vision-column")).toBeVisible();
      await expect(page.locator(".conversation")).toBeVisible();
      await page.screenshot({ path: `test-results/responsive/${width}-cameras.png`, fullPage: true, animations: "disabled" });
      await page.getByRole("button", { name: "Workspace", exact: true }).tap();
    }
    await page.locator("#chat-input").fill("status");
    await page.locator("#send").click();
    await expect(page.locator("#send")).toBeEnabled();
    expect(writes.some((r) => r.path === "/api/chat" && r.body.text === "status")).toBe(true);
    await page.locator("#voice-language").selectOption("de");
    await page.locator("#sound").click();
    await expect(page.locator("#sound")).toHaveAttribute("aria-pressed", "true");
    await page.locator("#sound").click();
    await expect(page.locator("#sound")).toHaveAttribute("aria-pressed", "false");
    await page.locator(".device-menu summary").click();
    await expect(page.locator("#pair")).toBeVisible();
    await page.locator("#pair").click();
    await expect(page.locator("#pair-dialog")).toBeVisible();
    await page.locator("#pair-dialog .close").click();
    await page.locator("#stop").click();
    expect(writes.some((r) => r.path === "/api/control" && r.body.action === "stop")).toBe(true);
    offline = true;
    await expect(page.locator("#connection")).toHaveText("Disconnected");
    await expect(page.locator("#talk")).toBeDisabled();
    await expect(page.locator("#resume")).toBeDisabled();
    await expect(page.locator("#stop")).toBeEnabled();
    const overflow = await page.evaluate(() => document.documentElement.scrollWidth > innerWidth);
    expect(overflow).toBe(false);
    expect(errors).toEqual([]);
    results.push({ width, height, overflow, errors, mockedWrites: writes.length });
    await context.close();
  }
  await writeFile("test-results/responsive/results.json", JSON.stringify(results, null, 2));
  console.log({ passed: true, robotCommandsSent: false, results });
} finally {
  await browser.close();
}
