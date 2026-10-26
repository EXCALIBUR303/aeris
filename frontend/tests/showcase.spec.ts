import { test, expect } from "@playwright/test";
import AxeBuilder from "@axe-core/playwright";
import { mkdirSync } from "node:fs";
const errors: string[] = [];
test.beforeEach(async ({ page }) => {
  errors.length = 0;
  page.on("pageerror", (e) => errors.push(e.message));
  page.on("console", (e) => {
    if (e.type() === "error") errors.push(e.text());
  });
  await page.goto("/");
  await expect(
    page.getByRole("heading", { name: "Autonomy, observed." }),
  ).toBeVisible();
});
test.afterEach(() => {
  expect(errors).toEqual([]);
});
for (const width of [360, 768, 1440])
  test(`responsive ${width}px, accessible controls and screenshot`, async ({
    page,
  }) => {
    await page.setViewportSize({ width, height: 1000 });
    await page.emulateMedia({ reducedMotion: "reduce" });
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth <= innerWidth,
      ),
    ).toBe(true);
    await page.waitForTimeout(600); // Audit settled UI, after view/theme choreography.
    const results = await new AxeBuilder({ page })
      .withTags(["wcag2a", "wcag2aa", "wcag21aa"])
      .analyze();
    expect(results.violations).toEqual([]);
    mkdirSync("public/screenshots", { recursive: true });
    await page.screenshot({
      path: `public/screenshots/mission-${width}.png`,
      fullPage: true,
    });
  });
test("play, pause, seek, speed, layer toggles and zoom", async ({ page }) => {
  await page.getByRole("button", { name: "Play replay", exact: true }).click();
  await expect(
    page.getByRole("button", { name: "Pause replay", exact: true }),
  ).toBeVisible();
  await page.waitForTimeout(1200);
  await page.getByRole("button", { name: "Pause replay", exact: true }).click();
  await page.getByRole("slider", { name: "Mission timeline" }).fill("180");
  await expect(page.getByRole("slider")).toHaveValue("180");
  await page.getByLabel("Playback", { exact: true }).selectOption("4");
  await page.getByRole("button", { name: "Route", exact: true }).click();
  await expect(
    page.getByRole("button", { name: "Route", exact: true }),
  ).toHaveAttribute("aria-pressed", "false");
  await page
    .getByRole("button", { name: "Sensor footprint", exact: true })
    .click();
  await page.getByRole("button", { name: "Zoom in", exact: true }).click();
  await expect(
    page.getByRole("button", { name: "Zoom out", exact: true }),
  ).toBeEnabled();
  await page.getByRole("button", { name: /Point A.*Reached/ }).click();
  await expect(page.getByRole("slider")).toHaveValue("84");
  await page.getByRole("button", { name: "Restart replay" }).click();
  await expect(page.getByRole("slider")).toHaveValue("0");
});
test("events filter, empty state, scenario change and linked charts", async ({
  page,
}) => {
  await page.getByRole("slider").fill("0");
  await page.getByLabel("Filter events").selectOption("target");
  await expect(page.getByText("No events yet", { exact: true })).toBeVisible();
  await page.getByRole("button", { name: "Show all events" }).click();
  await page.getByLabel("Environment", { exact: true }).selectOption("office");
  await expect(page.getByRole("slider")).toHaveValue("0");
  await expect(
    page.getByRole("img", { name: /Office 12 local ENU/ }),
  ).toBeVisible();
  await page.getByRole("button", { name: "Battery", exact: true }).click();
  await expect(page.getByRole("img", { name: /battery over 4/ })).toBeVisible();
});
test("palette keyboard, library search, theme and focus restoration", async ({
  page,
}) => {
  await page.keyboard.press("Control+k");
  await expect(page.getByRole("dialog")).toBeVisible();
  await page.getByPlaceholder("Search views, worlds, actions…").fill("library");
  await page.keyboard.press("Enter");
  await expect(
    page.getByRole("heading", { name: "A world of possibilities." }),
  ).toBeVisible();
  await page.getByLabel("Search environments").fill("does-not-exist");
  await expect(page.getByText("No worlds found")).toBeVisible();
  await page.getByRole("button", { name: "Clear search" }).click();
  await page.getByRole("button", { name: /02 \/ SEEDED WORLD/ }).click();
  await expect(
    page.getByRole("heading", { name: "Autonomy, observed." }),
  ).toBeVisible();
  await page.getByRole("button", { name: "Switch to dark theme" }).click();
  await page.waitForTimeout(600); // Audit settled UI, after view/theme choreography.
  const results = await new AxeBuilder({ page })
    .withTags(["wcag2a", "wcag2aa", "wcag21aa"])
    .analyze();
  expect(results.violations).toEqual([]);
  await page.screenshot({
    path: "public/screenshots/mission-dark.png",
    fullPage: true,
  });
  await page.getByRole("button", { name: "About this demo" }).click();
  await page.keyboard.press("Escape");
  await expect(page.getByRole("dialog")).not.toBeVisible();
  await expect(
    page.getByRole("button", { name: "About this demo" }),
  ).toBeFocused();
});
test("export provenance and analytics", async ({ page }) => {
  const download = page.waitForEvent("download");
  await page.getByRole("button", { name: "Export snapshot" }).click();
  expect((await download).suggestedFilename()).toContain(
    "aeris-warehouse-128s",
  );
  await expect(page.getByRole("status")).toContainText("Snapshot exported");
  await page.getByRole("button", { name: "Analytics", exact: true }).click();
  await expect(page.getByText("Worlds, side by side.")).toBeVisible();
  await page.screenshot({
    path: "public/screenshots/analytics.png",
    fullPage: true,
  });
});
