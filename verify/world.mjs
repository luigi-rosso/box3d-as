import { runModule } from './lib.mjs';

// The world lives in module globals; a collection runs between steps, so it is moved mid run.
await runModule('world', (e, ctx) => {
  e.registry();
  ctx.frameBoundary();
  e.createScene();
  ctx.frameBoundary();
  for (let i = 0; i < 160; ++i) {
    e.step(i);
    ctx.frameBoundary();
  }
  e.finish();
});
