import { runModule } from './lib.mjs';

await runModule('compound', (e, ctx) => {
  e.buildPools();
  ctx.frameBoundary();
  for (let i = 0; i < 60; ++i) {
    e.runCompound(i);
    ctx.frameBoundary();
  }
  e.tooMany();
});
