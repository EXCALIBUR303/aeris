import { test } from "node:test";
import assert from "node:assert/strict";
import {
  makeScenarios,
  generateFrames,
  positionAt,
  recordings,
  scenarios,
  snapshot,
} from "../src/modules/aeris/data";
import { makeStore } from "../src/modules/aeris/store";
test("regeneration is deterministic and a different seed changes geometry", () => {
  assert.deepEqual(makeScenarios(), makeScenarios(303));
  assert.notDeepEqual(makeScenarios(304), makeScenarios());
});
test("all frames stay finite, bounded, monotonic, and outside obstacles", () => {
  for (const s of scenarios) {
    const frames = generateFrames(s);
    assert.equal(frames.length, 241);
    frames.forEach((f, i) => {
      for (const value of Object.values(f)) assert.ok(Number.isFinite(value));
      assert.ok(f.coverage >= 0 && f.coverage <= 100);
      assert.ok(f.battery > 0 && f.battery <= 100);
      if (i) {
        assert.ok(f.distance >= frames[i - 1].distance);
        assert.ok(f.coverage >= frames[i - 1].coverage);
        assert.ok(f.battery <= frames[i - 1].battery);
      }
      assert.ok(
        !s.obstacles.some(
          (o) => f.x > o.x && f.x < o.x + o.w && f.y > o.y && f.y < o.y + o.h,
        ),
      );
    });
    for (const t of s.targets) {
      const p = positionAt(s, t.time);
      assert.equal(t.x, p.x);
      assert.equal(t.y, p.y);
    }
  }
});
test("export is explicitly synthetic and contains no future events", () => {
  const s = scenarios[0];
  const exportData = snapshot(s, 80);
  assert.equal(exportData.scientificEvidence, false);
  assert.equal(exportData.provenance, "synthetic-demo");
  assert.deepEqual(exportData.frame, recordings[s.id][80]);
  assert.ok(exportData.events.every((e) => e.time <= 80));
});
test("clock clamps, finishes, restarts and scenario changes stop playback", () => {
  const store = makeStore();
  store.getState().seek(-100);
  assert.equal(store.getState().time, 0);
  store.getState().seek(999);
  assert.equal(store.getState().time, 240);
  store.getState().toggle();
  assert.equal(store.getState().time, 0);
  store.getState().seek(239.9);
  store.getState().toggle();
  store.getState().tick();
  assert.equal(store.getState().time, 240);
  assert.equal(store.getState().playing, false);
  store.getState().setScenario("office");
  assert.equal(store.getState().time, 0);
  assert.equal(store.getState().playing, false);
});
test("embeds do not share store state", () => {
  const first = makeStore();
  const second = makeStore();
  first.getState().setTheme("dark");
  first.getState().seek(22);
  assert.equal(second.getState().theme, "light");
  assert.equal(second.getState().time, 128);
});
