import { runModule } from './lib.mjs';

// The world stays in module globals; a collection between steps moves it, as a long run would.
// CONTACT_SCENES=4 runs only the scripted scene, for vectors from the CONTACT_COLLIDE_ONLY C build.
const only = process.env.CONTACT_SCENES ? process.env.CONTACT_SCENES.split(',').map(Number) : null;

await runModule('contact', (e, ctx) => {
  const sceneCount = e.sceneCount();
  for (let s = 0; s < sceneCount; ++s) {
    if (only && !only.includes(s)) continue;
    e.begin(s);
    ctx.frameBoundary();
    const steps = e.sceneSteps(s);
    for (let i = 0; i < steps; ++i) {
      e.step(i);
      ctx.frameBoundary();
    }
    e.finish();
    ctx.frameBoundary();
  }
});
