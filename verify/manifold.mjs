import { runModule } from './lib.mjs';

await runModule('manifold', (e, ctx) => {
  e.begin();
  ctx.frameBoundary();
  e.run();
});
