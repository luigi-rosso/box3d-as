import { runModule } from './lib.mjs';

// One export call per phase, in the order of verify/cref/hull.c, with a collection after each.
await runModule('hull', (e, ctx) => {
  const step = (f) => {
    f();
    ctx.frameBoundary();
  };
  const sets = e.setCount();
  for (let set = 0; set < sets; ++set) step(() => e.phaseSample(set));
  step(() => e.phaseWheel());
  for (let k = 0; k < 720; ++k) step(() => e.phaseRandom(k));
  for (let k = 0; k < 60; ++k) step(() => e.phaseShapes(k));
  for (let k = 0; k < 60; ++k) step(() => e.phaseBoxes(k));
  step(() => e.phaseMap());
  for (let k = 0; k < 400; ++k) step(() => e.phase2D(k));
  for (let k = 0; k < 8; ++k) step(() => e.phaseLimits(k));
  for (let k = 0; k < 200; ++k) step(() => e.phaseValidate(k));
});
