import { runModule } from './lib.mjs';

// A collection between phases moves the set, as a long simulation would.
await runModule('table', (e, ctx) => {
  e.begin();
  for (let round = 0; round < 6; ++round) {
    e.beginRound(round);
    for (let phase = 0; phase < 12; ++phase) {
      e.phase();
      ctx.frameBoundary();
    }
    e.endRound();
    ctx.frameBoundary();
  }
});
