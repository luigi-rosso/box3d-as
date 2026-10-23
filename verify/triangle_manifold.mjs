import { runModule } from './lib.mjs';

// One export call per phase, in the order of verify/cref/triangle_manifold.c, with a collection between.
await runModule('triangle_manifold', (e, ctx) => {
  const step = (f) => {
    f();
    ctx.frameBoundary();
  };
  step(() => e.begin());
  step(() => e.runSpheres(20000));
  for (let batch = 0; batch < 4; ++batch) step(() => e.runCapsules(2000));
  for (let batch = 0; batch < 10; ++batch) step(() => e.runHulls(batch, 1000));
  e.setLengthUnits(0.37);
  step(() => e.runHulls(100, 1000));
  step(() => e.runCapsules(1000));
  e.setLengthUnits(1.0);
});
