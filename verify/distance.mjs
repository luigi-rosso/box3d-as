import { runModule } from './lib.mjs';

await runModule('distance', (e, ctx) => {
  e.begin();
  e.runFixed();
  e.runSupport();
  e.runSweep();
  ctx.frameBoundary();
  for (const run of [e.runDistance, e.runCast, (n) => e.runToi(n, -1)]) {
    for (let batch = 0; batch < 8; ++batch) {
      run(1000);
      ctx.frameBoundary();
    }
  }
  e.setLengthUnits(0.37);
  e.runCast(1000);
  ctx.frameBoundary();
  e.runToi(1000, -1);
  e.setLengthUnits(1.0);
  ctx.frameBoundary();
  for (let batch = 0; batch < 4; ++batch) {
    e.runToi(5000, 5);
    ctx.frameBoundary();
  }
});
