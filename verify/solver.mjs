import { runModule } from './lib.mjs';

// One world per scene lives in module globals; a collection runs between steps, so it is moved mid run.
await runModule('solver', (e, ctx) => {
  for (let scene = 0; scene < 4; ++scene) {
    e.startScene(scene);
    ctx.frameBoundary();
    const steps = e.stepCount(scene);
    for (let i = 0; i < steps; ++i) {
      e.stepScene(i);
      ctx.frameBoundary();
    }
    e.endScene();
    ctx.frameBoundary();
  }
});
