import { runModule } from './lib.mjs';

// The tree lives in a module global and a collection runs between phases, so it is moved mid run.
await runModule('tree', (e, ctx) => {
  e.scripted();
  ctx.frameBoundary();
  for (let round = 0; round < 12; ++round) {
    e.beginRound(round);
    ctx.frameBoundary();
    for (let phase = 0; phase < 10; ++phase) {
      e.phase();
      ctx.frameBoundary();
    }
    e.endRound();
    ctx.frameBoundary();
  }
});
