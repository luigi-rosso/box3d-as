import { runModule } from './lib.mjs';

await runModule('bodyshape', (e, ctx) => {
  const phases = [
    () => e.phaseCreate(),
    () => e.phaseQueries(),
    () => e.phaseMutate(),
    () => e.phaseQueries(),
    () => e.phaseStep(10),
    () => e.phaseStep(20),
    () => e.phaseQueries(),
    () => e.phaseStep(60),
    () => e.phaseExtras(),
    () => e.phaseExtrasStep(),
    () => e.phaseExtrasJoints(),
  ];
  for (const phase of phases) {
    phase();
    ctx.frameBoundary();
  }
});
