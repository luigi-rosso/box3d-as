import { runModule } from './lib.mjs';

// One export call per phase, in the order of verify/cref/convex_manifold.c, with a collection between.
await runModule('convex_manifold', (e, ctx) => {
  const step = (f) => {
    f();
    ctx.frameBoundary();
  };
  step(() => e.begin());
  step(() => e.runSpheres(6000));
  step(() => e.runCapsuleSphere(6000));
  step(() => e.runCapsules(12000));
  for (let batch = 0; batch < 4; ++batch) step(() => e.runHullSphere(1000));
  for (let batch = 0; batch < 6; ++batch) step(() => e.runHullCapsule(1000));
  step(() => e.runSlide(0));
  for (let batch = 0; batch < 8; ++batch) step(() => e.runHulls(batch, 500));
  e.setLengthUnits(0.37);
  step(() => e.runSlide(1));
  step(() => e.runHulls(100, 500));
  e.setLengthUnits(1.0);
});
