import { runModule } from './lib.mjs';

// The world lives in module globals and a collection runs after every step, so it is moved mid run.
await runModule('joints', (e, ctx) => {
  e.setup();
  ctx.frameBoundary();
  const n = e.stepCount();
  for (let i = 0; i < n; ++i) {
    e.step(i);
    ctx.frameBoundary();
  }
  e.teardown();
});
