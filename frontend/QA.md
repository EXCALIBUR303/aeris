# Verification evidence

Validated 2026-10-26 on the production build, not the development server. Node 24.19.0, Next.js 16.4.3, Python 3.12.12, system Chromium, Lighthouse 13.5.0.

| Check                    | Result                                                                                                                                          |
| ------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| `npm run lint`           | Pass, no warnings                                                                                                                               |
| `npm run typecheck`      | Pass                                                                                                                                            |
| `npm test`               | 5 passed: deterministic fixtures, bounded/finite telemetry and obstacle-free routes, export provenance, clock behavior, independent embed state |
| `npm run build`          | Pass; statically generated route, icon and robots.txt                                                                                           |
| `npm run test:e2e`       | **7 passed**, run after the final Lighthouse measurement; zero page/console errors across the tested flows                                      |
| Responsive checks        | 360, 768, 1440px; no horizontal overflow; full-page screenshots captured                                                                        |
| axe WCAG 2 A/AA + 2.1 AA | Zero violations in tested mission layouts and dark theme, including text contrast                                                               |
| Keyboard                 | Command palette search/Enter, native slider, dialog Escape and focus restoration verified                                                       |
| Python CI unit workflow  | **1,040 passed, 2 skipped**; existing checks were reused rather than rerun after UI-only optimizations                                          |
| Python quality checks    | Ruff lint/format passed, mypy passed on 150 files, 9 import contracts kept                                                                      |

## Final mobile Lighthouse

[Raw production report](qa/lighthouse-mobile.json), measured before rerunning the seven browser tests so browser-test contention did not affect the audit.

| Category       | Score   |
| -------------- | ------- |
| Performance    | **94**  |
| Accessibility  | **100** |
| Best practices | **100** |
| SEO            | **100** |

Largest Contentful Paint: **2.0 s**. Total Blocking Time: **260 ms**. Cumulative Layout Shift: **0**. Mobile simulation: 4× CPU slowdown, 150 ms RTT, 1,638.4 Kbps throughput. These are laboratory results from this machine, not field measurements or a guarantee for every host.

Reproduce from `frontend/` with the production server running:

```bash
node_modules/.bin/lighthouse http://127.0.0.1:3000 \
  --chrome-path=/usr/bin/chromium \
  --chrome-flags='--headless --no-sandbox' \
  --output=json --output-path=qa/lighthouse-mobile.json \
  --only-categories=performance,accessibility,best-practices,seo
npm run test:e2e
```

Startup improvements: command palette and guide are separate lazy chunks loaded only on opening; optional Motion layout/animation features load on first pointer/key interaction; initial content is server-rendered rather than hidden behind a hydration fade. The fixture generator uses squared-distance checks and compact visited flags, avoids recomputing previous positions and produces exactly the same saved frames (deep equality verified). System fonts require no font downloads; the mission diagrams, drone and icons are SVG, with no runtime raster images, map tiles or third-party asset requests. Screenshots in this documentation do not load in the application.

## Accessibility and visual evidence

Primary text contrast: light 15.30:1, dark 14.12:1. Secondary text against the page/surface: light 4.62:1, dark 7.44:1. Controls retain visible focus; plots/maps have text descriptions and values, and the range input provides the keyboard equivalent of scrubbing. System reduced-motion mode is exercised during responsive tests; ornamental scan animation and count interpolation respect the preference. Automated checks and keyboard testing are not a formal screen-reader conformance certification.

Screenshots: [360px](public/screenshots/mission-360.png), [768px](public/screenshots/mission-768.png), [1440px](public/screenshots/mission-1440.png), [dark theme](public/screenshots/mission-dark.png), [analytics](public/screenshots/analytics.png). [Playwright machine-readable results](qa/playwright.json) record the final seven tests.

## Diagnosed limits

- The two Python skips require external PX4/Gazebo. A non-CI initial run hit the documented Gazebo-dependent timeout test because `gz` is absent; the repository's existing `CI=1` guard correctly skips it. No backend tests or assertions were weakened. Existing FastSim NumPy warnings remain research-runtime work.
- The disconnected build flag and render-recovery boundary are implemented; the seven browser tests cover demo mode, not a separate disabled-mode build or injected render crashes.
- Real backend transport, recorded-run conversion, authenticated vehicle commands, sensor images, simulator installation and live-vehicle operation were not implemented or validated. No API contracts changed.
- Testing used Chromium; Safari, Firefox, manual screen-reader testing and a deployed portfolio host remain unverified. The module targets a portfolio route; narrow containers on wide screens need an iframe or future container-query adaptation.
