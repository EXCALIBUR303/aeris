# ADR-0013: React/TS/Vite + CSS-variable tokens + Radix + R3F + uPlot + Observable Plot; no Bootstrap

**Status:** Accepted (Phase 0), to be confirmed in Phase 26
**Spec reference:** AERIS_TECHNICAL_SPEC.md §46.1

## Context

The prompt that seeded this project suggested Bootstrap 5 "only where
genuinely useful." AERIS's design language (§47–48: avionics/engineering
aesthetic, precise semantic-color and provenance encoding) is
purpose-specific enough that a general-purpose component framework's
defaults would fight it more than help.

## Decision

React + TypeScript + Vite; styling via CSS Modules and design tokens as CSS
custom properties (`design/tokens/*.json`, Phase 26); Radix UI primitives
for accessible unstyled interaction components; TanStack Query for server
state; Zustand for client state; three.js/React Three Fiber for the 3D
mission view; uPlot for realtime telemetry; Observable Plot (fallback
visx) for scientific analytics charts (CI bands, facets, log scales).
**Bootstrap is not used.**

## Consequences

Full control over the visual language required by §47–48, at the cost of
building more UI primitives from scratch than a component-library-first
approach would need. Phase 26 (professional product/UI design) makes the
final call and may refine this list once real screen designs exist.
