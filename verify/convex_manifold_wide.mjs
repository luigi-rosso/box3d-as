import { runModule } from './lib.mjs';

// The hull phases of convex_manifold.mjs, built with the lane loop for edge pairs.
await runModule('convex_manifold_wide', (e, ctx) => {
  const step = (f) => {
    f();
    ctx.frameBoundary();
  };
  step(() => e.begin());
  step(() => e.runSlide(0));
  for (let batch = 0; batch < 8; ++batch) step(() => e.runHulls(batch, 500));
  e.setLengthUnits(0.37);
  step(() => e.runSlide(1));
  step(() => e.runHulls(100, 500));
  e.setLengthUnits(1.0);
});
