import { writeFileSync, mkdirSync } from "node:fs";
import { makeScenarios, generateFrames } from "../src/modules/aeris/data";
mkdirSync("public/demo", { recursive: true });
writeFileSync(
  "public/demo/seed-303.json",
  JSON.stringify(
    {
      provenance: "synthetic-demo",
      seed: 303,
      scenarios: makeScenarios().map((s) => ({
        ...s,
        frames: generateFrames(s),
      })),
    },
    null,
    2,
  ) + "\n",
);
console.log(
  "Regenerated deterministic seed-303.json (illustrative, not measured research).",
);
