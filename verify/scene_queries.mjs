import { runModule } from './lib.mjs';

// The scene lives in a module global; a collection runs between steps.
await runModule('scene_queries', (e, ctx) => {
  const steps = [ 200, 40 ];
  for ( let s = 0; s < 2; ++s ) {
    e.create( s );
    ctx.frameBoundary();
    for ( let i = 0; i < steps[s]; ++i ) {
      e.step( i );
      ctx.frameBoundary();
    }
    e.destroy();
    ctx.frameBoundary();
  }
});
