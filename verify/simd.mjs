import { runModule } from './lib.mjs';

// cref/simd.flags builds the driver at -O0: at -O2 clang commutes multiplies,
// which changes the payload when both operands are NaN. Run with
// NAN_EQUAL=1 against an -O2 reference to check the rest of the bits.
await runModule('simd', (e) => e.run(), { nanEqual: process.env.NAN_EQUAL === '1' });
