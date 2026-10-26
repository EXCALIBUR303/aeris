# Decisions to review

| Decision | Alternatives | Why | How to reverse |
| --- | --- | --- | --- |
| Standalone Next.js host; portable `frontend/src/modules/aeris` | Vite per ADR-0013; integrate nonexistent server | User requested Next portfolio; preserve original runtime/ADR | Copy module to another React host |
| Demo on by build flag; false shows disconnected state | Invent an API; silent fake fallback | No server exists; never imply live telemetry | Add reviewed transport adapter and provenance |
| Seed 303 synthetic example missions | Require PX4; fabricate scientific evidence | Deterministic, safe, clearly illustrative visitor experience | Replace generator with validated recorded-run export |
| SVG 2D map and plots; no Three.js | R3F; canvas map library | Coverage, route and timing are 2D questions; less GPU cost | Lazy-load 3D only when real height/voxel data adds value |
| Controls only affect local playback | Public arm/land commands | Portfolio must never command a vehicle | Separate safety-reviewed authenticated command API |
| Light instrument/editorial style, dark theme, scoped CSS tokens | Dark only; global reset | Readability and no embedding style leakage | Change module tokens |
| Local shadcn-style Button/Dialog using Radix | Download registry template; large kit | Auditable accessible primitives, no downloaded commands | Replace `ui/` components |
| Local fonts and SVG; no external images/tiles | Hosted fonts/maps | Offline demo, predictable privacy and loading | Add optional host assets and fallbacks |
| Current work branch, feature commits | Worktree/new branch | Explicit instruction | Revert/cherry-pick individual commits |
