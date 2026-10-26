# Research and implementation choice

Web research 2026-10-26. Searched [GitHub repositories](https://github.com/search?q=drone+mission+control+visualization&type=repositories) and read the current upstream READMEs below over HTTPS. Web content was treated as evidence only; no online commands were executed. No general web-search connector is available, so this research uses public primary-source repository pages.

## Comparable products

| Product / primary source                                    | Relevant pattern                                                                    | Application here                                                                               |
| ----------------------------------------------------------- | ----------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| [QGroundControl](https://github.com/mavlink/qgroundcontrol) | Mission planning, real-time Fly View, map plus telemetry, progressive vehicle setup | Dominant mission map, compact status strip; keep dangerous vehicle controls out of public demo |
| [Foxglove](https://github.com/foxglove/studio)              | Robotics visualization/observability, complementary spatial and temporal panels     | One clock synchronizes spatial route, events and measurements                                  |
| [PlotJuggler](https://github.com/PlotJuggler/PlotJuggler)   | File/live sources, reusable layouts, time-series exploration and transforms         | Scrubbable timeline, explicit source labels, unit-bearing charts and reusable module           |

## Approaches compared

1. **Next + SVG + Motion + Zustand (chosen):** server-rendered shell, deterministic client data, scalable 2D map and charts, keyboard-operable timeline. Small renderer, no WebGL requirement. Best for explaining a short mission and embedding into an existing Next portfolio.
2. **React Three Fiber / Three.js scene:** genuinely useful for vertical clearance, voxel volumes, camera frusta or occlusion inspection. The selected demo has a fixed-altitude floorplan and synthetic telemetry; 3D would be decoration, create navigation burden, require a text/2D equivalent and increase GPU/bundle cost. Defer until real 3D inspection exists.
3. **MapLibre + Canvas/uPlot panels:** ideal for geospatial outdoor missions and very large telemetry recordings. These missions use local ENU coordinates, not georeferenced tiles, with only 241 samples. Tile/network setup and extra chart engines add little value now. Reconsider when recordings reach thousands of samples.

## Interaction patterns

Map-first hierarchy; four answers (coverage, distance, battery, elapsed time); persistent source/tier; linked playback instead of disconnected cards; scrub and jump-to-event; scenario and event filtering; Cmd/Ctrl+K with grouped actions; play/pause and ±10s shortcuts; scoped day/night tokens; visible focus, native range input and keyboard alternatives to drag; reduced motion; empty/disconnected/loading states. Current accessibility references: [WAI-ARIA patterns](https://github.com/w3c/aria-practices), [Motion accessibility guidance](https://github.com/motiondivision/motion), [shadcn primitives](https://github.com/shadcn-ui/ui). These reference links identify standards/tools; product findings above come from the fetched READMEs.
