import { runModule } from './lib.mjs';

// STAGE=1 runs only the queries that need no GJK; the C driver takes the same stage argument.
const stage = Number(process.env.STAGE ?? 3);

await runModule('mesh', (e, ctx) => {
  const meshCount = e.buildAll();
  ctx.frameBoundary();
  for (const [bit, run] of [[1, e.queries], [2, e.casts]]) {
    if ((stage & bit) === 0) continue;
    for (let m = 0; m < meshCount; ++m) {
      for (let s = 0; s < 6; ++s) {
        run(m, s);
        ctx.frameBoundary();
      }
    }
  }
});
