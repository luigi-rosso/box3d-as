import { runModule } from './lib.mjs';

// The C driver is built at -O2 as the reference build is, and clang commutes operations whose
// operands are both NaN, so NaN payloads and signs are not part of the contract here.
await runModule('math', (e) => {
  e.run();
  if (e.instantiate() !== 0) {
    throw new Error('instantiate check failed');
  }
}, { nanEqual: process.env.NAN_EXACT !== '1' });
