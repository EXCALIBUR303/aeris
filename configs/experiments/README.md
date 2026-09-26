# AERIS experiment configs

Spec §38.1: "a declared matrix (methods × seeds × world splits ×
conditions) ... expanded into runs by `aeris.experiments.runner`."

No real matrix axis exists to expand yet as of Phase 7 — there's one
method (`scripted`, `MissionExecutive`), no world splits (Phase 33), and
no randomization a seed would meaningfully vary. `aeris.experiments.runner.run_evaluation()`
is the real, live-verified building block (one run in, one manifest +
MCAP + metrics out); a matrix-expansion layer on top of it is only worth
building once Phase 9+ gives it real axes to expand across.

`preregistration/<exp_id>.md` is where a committed pre-registration for a
**test-split** evaluation lives (spec §38.5) — `aeris.experiments.preregistration.require_preregistration()`
refuses without one. Empty as of Phase 7: no test split exists yet
(world splits are Phase 33's job).
