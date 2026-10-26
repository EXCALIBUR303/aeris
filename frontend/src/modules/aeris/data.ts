/** Illustrative fixtures only. Never use as scientific measurements or vehicle commands. */
export type Point = { x: number; y: number };
export type MissionEvent = {
  time: number;
  kind: "system" | "target" | "navigation";
  title: string;
  detail: string;
};
export type Scenario = {
  id: string;
  name: string;
  family: string;
  seed: number;
  duration: number;
  route: Point[];
  obstacles: { x: number; y: number; w: number; h: number }[];
  targets: (Point & { time: number; name: string })[];
  events: MissionEvent[];
};
export type Frame = Point & {
  time: number;
  distance: number;
  coverage: number;
  battery: number;
  speed: number;
  altitude: number;
};
export const formatTime = (n: number) =>
  `${Math.floor(n / 60)
    .toString()
    .padStart(2, "0")}:${Math.floor(n % 60)
    .toString()
    .padStart(2, "0")}`;
export function seeded(seed: number) {
  let s = seed >>> 0;
  return () => {
    s = (1664525 * s + 1013904223) >>> 0;
    return s / 4294967296;
  };
}
export function makeScenarios(seed = 303): Scenario[] {
  const random = seeded(seed);
  return ["Warehouse 03", "Office 12", "Rubble field 07"].map((name, index) => {
    const obstacles = Array.from({ length: index === 2 ? 10 : 8 }, (_, i) => ({
      x: 145 + (i % 4) * 130 + (index === 2 ? random() * 15 : 0),
      y: 116 + Math.floor(i / 4) * 130,
      w: index === 1 ? 82 : 68 + random() * 12,
      h: 70,
    }));
    const route = [
      { x: 85, y: 355 },
      { x: 85, y: 80 },
      { x: 625, y: 80 },
      { x: 625, y: 215 },
      { x: 85, y: 215 },
      { x: 85, y: 355 },
      { x: 625, y: 355 },
      { x: 625, y: 215 },
      { x: 405, y: 215 },
    ];
    const scenario: Scenario = {
      id: ["warehouse", "office", "rubble"][index],
      name,
      family: ["F3 · Warehouse", "F2 · Office", "F1 · Rubble"][index],
      seed: seed + index,
      duration: 240,
      route,
      obstacles,
      targets: [
        { x: 625, y: 150, time: 84, name: "Inspection point A" },
        { x: 330, y: 355, time: 178, name: "Inspection point B" },
      ],
      events: [
        {
          time: 0,
          kind: "system",
          title: "Replay initialized",
          detail: "Synthetic fixture · CPU · seed " + (seed + index),
        },
        {
          time: 12,
          kind: "navigation",
          title: "Exploration started",
          detail: "Frontier-based route · ENU frame",
        },
        {
          time: 62,
          kind: "navigation",
          title: "North aisle mapped",
          detail: "Occupancy updated in local map",
        },
        {
          time: 84,
          kind: "target",
          title: "Inspection point A reached",
          detail: "Illustrative mission annotation",
        },
        {
          time: 128,
          kind: "navigation",
          title: "New frontier selected",
          detail: "Continuing through central aisle",
        },
        {
          time: 178,
          kind: "target",
          title: "Inspection point B reached",
          detail: "Illustrative mission annotation",
        },
        {
          time: 240,
          kind: "system",
          title: "Replay complete",
          detail: "Export a snapshot to inspect the fixture",
        },
      ],
    };
    scenario.targets = scenario.targets.map((t) => ({
      ...t,
      ...positionAt(scenario, t.time),
    }));
    return scenario;
  });
}
export const scenarios = makeScenarios();
function length(a: Point, b: Point) {
  return Math.hypot(b.x - a.x, b.y - a.y);
}
export function positionAt(s: Scenario, time: number): Point {
  const segments = s.route.slice(1).map((p, i) => length(s.route[i], p));
  let remaining =
    segments.reduce((a, b) => a + b, 0) *
    Math.max(0, Math.min(time / s.duration, 1));
  for (let i = 0; i < segments.length; i++) {
    if (remaining <= segments[i]) {
      const r = remaining / segments[i];
      return {
        x: s.route[i].x + (s.route[i + 1].x - s.route[i].x) * r,
        y: s.route[i].y + (s.route[i + 1].y - s.route[i].y) * r,
      };
    }
    remaining -= segments[i];
  }
  return s.route[s.route.length - 1];
}
export function generateFrames(s: Scenario): Frame[] {
  const cells: Point[] = [];
  for (let x = 55; x < 670; x += 20)
    for (let y = 55; y < 390; y += 20)
      if (
        !s.obstacles.some(
          (o) => x >= o.x && x <= o.x + o.w && y >= o.y && y <= o.y + o.h,
        )
      )
        cells.push({ x, y });
  const seen = new Uint8Array(cells.length);
  let seenCount = 0;
  let distance = 0;
  let previous = s.route[0];
  return Array.from({ length: s.duration + 1 }, (_, time) => {
    const p = positionAt(s, time);
    const stepDistance = time ? length(previous, p) / 10 : 0;
    distance += stepDistance;
    previous = p;
    cells.forEach((c, i) => {
      if (seen[i]) return;
      const dx = c.x - p.x,
        dy = c.y - p.y;
      if (dx * dx + dy * dy <= 83 * 83) {
        seen[i] = 1;
        seenCount++;
      }
    });
    return {
      ...p,
      time,
      distance,
      coverage: (seenCount / cells.length) * 100,
      battery: 100 - distance * 0.075,
      speed: time === 0 || time === s.duration ? 0 : stepDistance,
      altitude: time === 0 ? 0 : 2.4 + Math.sin(time / 18) * 0.04,
    };
  });
}
export const recordings = Object.fromEntries(
  scenarios.map((s) => [s.id, generateFrames(s)]),
);
export function snapshot(s: Scenario, time: number) {
  return {
    schemaVersion: 1,
    provenance: "synthetic-demo",
    scientificEvidence: false,
    seed: s.seed,
    scenario: s.name,
    frame:
      recordings[s.id][Math.max(0, Math.min(s.duration, Math.floor(time)))],
    events: s.events.filter((e) => e.time <= time),
  };
}
