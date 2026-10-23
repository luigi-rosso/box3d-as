import { runModule } from './lib.mjs';

await runModule('mesh_contact', (e, ctx) => {
  e.buildPools();
  ctx.frameBoundary();
  for (let i = 0; i < 120; ++i) {
    e.runSequence(i);
    ctx.frameBoundary();
  }
});
