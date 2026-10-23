import { runModule } from './lib.mjs';

await runModule('sphere', (e, ctx) => {
  e.begin();
  for (const run of [e.runBasics, e.runOverlap, e.runRayCast, e.runShapeCast, e.runMover]) {
    run();
    ctx.frameBoundary();
  }
});
