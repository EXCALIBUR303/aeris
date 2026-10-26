# AERIS showcase delivery

Built a separate Next.js App Router + TypeScript + Tailwind + local shadcn/Radix + Motion + Zustand showcase. It includes three seeded worlds, linked map/telemetry/events, playback/scrubbing/speed, layer and zoom controls, inspection jumps, event filters, mission library search, analytics, JSON export, themes, keyboard commands, loading/empty/disconnected/recovery states, and reduced motion. The exportable module is `frontend/src/modules/aeris/`. No Python application logic or API contracts changed.

The production mobile Lighthouse result is **94 / 100 / 100 / 100** (performance/accessibility/best practices/SEO). All **7 browser tests passed afterward**, with zero console errors and 360/768/1440px screenshots. Five data/store tests, lint, typecheck and build pass. Existing Python validation: 1,040 tests passed, two simulator-dependent skips; Ruff, mypy and all nine import contracts passed. See [QA evidence](frontend/QA.md).

[README, screenshots, architecture and run/embed instructions](frontend/README.md) are complete. [AUDIT.md](AUDIT.md) explains the original state; [RESEARCH.md](RESEARCH.md) links three comparables and compares SVG, R3F and map/chart-engine approaches. Changes are in feature commits on the original `work` branch.

## Every decision to review

All choices, alternatives, rationale and reversal paths are recorded in [DECISIONS.md](DECISIONS.md):

1. Standalone Next.js host and portable module, while preserving the research ADR's original Vite direction.
2. Demo enabled by build flag; disabling it shows disconnected status rather than inventing an API.
3. Seed 303 synthetic missions, explicitly distinguished from real evaluation.
4. SVG 2D map/charts; no Three.js until real vertical/voxel inspection justifies it.
5. Playback controls only; no public flight commands.
6. Light instrument-style design, dark theme and scoped CSS tokens.
7. Locally owned shadcn-style Button/Dialog built on Radix.
8. System fonts and local SVG, with no external image/map services.
9. Current branch and small feature commits, with no worktree or branch replacement.

## What remains

A live research console still needs server/WebSocket contracts, authenticated command authorization, MCAP-to-browser data adapters, sensor imagery and real simulator integration. Higher research roadmap phases remain as described in phase reports. No deployment target was provided, so the module is buildable and ready to embed but not published. Cross-browser and manual screen-reader testing, disabled-mode build/crash-injection tests, and container-query layouts remain outside the verified checks. The data is illustrative, never scientific evidence.

Run: `cd frontend && npm ci && npm run dev`. Production: `npm run build && npm start`.
