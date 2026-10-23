// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Shared node side of the parity harness: loads a rasc wasm, collects the
// tokens the driver emits through verify/as/common.as, and compares them with
// the lines the C reference printed through verify/cref/common.h.

import { readFileSync } from 'fs';
import { dirname, join } from 'path';
import { fileURLToPath } from 'url';

export const verifyDir = dirname(fileURLToPath(import.meta.url));

// f32 / f64 bit helpers. Bits are unsigned numbers (f32) or BigInts (f64).
const f32buf = new Float32Array(1);
const u32buf = new Uint32Array(f32buf.buffer);
const f64buf = new Float64Array(1);
const u64buf = new BigUint64Array(f64buf.buffer);

export function f32ToBits(x) { f32buf[0] = x; return u32buf[0]; }
export function bitsToF32(bits) { u32buf[0] = bits >>> 0; return f32buf[0]; }
export function f64ToBits(x) { f64buf[0] = x; return u64buf[0]; }
export function bitsToF64(bits) { u64buf[0] = BigInt.asUintN(64, BigInt(bits)); return f64buf[0]; }
export function hex32(bits) { return (bits >>> 0).toString(16).padStart(8, '0'); }
export function hex64(bits) { return BigInt.asUintN(64, BigInt(bits)).toString(16).padStart(16, '0'); }

// Reads a vector file into lines of tokens.
export function readVectors(path) {
  const text = readFileSync(path, 'utf8');
  const lines = text.split('\n');
  if (lines.length > 0 && lines[lines.length - 1] === '') lines.pop();
  return lines;
}

// Collects emitted lines. The driver calls the vec imports; a line ends at vend.
export class VectorSink {
  constructor() {
    this.lines = [];
    this.tokens = [];
    this.chars = [];
  }
  imports() {
    return {
      ch: (c) => {
        if (c === 0) {
          this.tokens.push(String.fromCharCode(...this.chars));
          this.chars.length = 0;
        } else {
          this.chars.push(c);
        }
      },
      i32: (x) => { this.tokens.push(String(x | 0)); },
      u32: (x) => { this.tokens.push(String(x >>> 0)); },
      f32: (bits) => { this.tokens.push(hex32(bits)); },
      u64: (x) => { this.tokens.push(hex64(x)); },
      end: () => { this.lines.push(this.tokens.join(' ')); this.tokens = []; },
    };
  }
}

// Instantiates a wasm with the vec imports and a stub for anything else. A
// stubbed import logs its first use, so a silent 0 never hides.
export function loadWasm(path, extraImports = {}) {
  const sink = new VectorSink();
  const module = new WebAssembly.Module(readFileSync(path));
  const warned = new Set();
  const stubModule = (moduleName) => new Proxy({}, {
    get: (_, name) => (...args) => {
      const key = `${moduleName}.${String(name)}`;
      if (!warned.has(key)) { warned.add(key); console.log(`note: stub import ${key}(${args.join(', ')})`); }
      return 0;
    },
  });
  const imports = new Proxy({ vec: sink.imports(), ...extraImports }, {
    get: (target, name) => (name in target ? target[name] : stubModule(String(name))),
  });
  const instance = new WebAssembly.Instance(module, imports);
  const exports = instance.exports;
  const frameBoundary = exports.frameBoundary ? () => exports.frameBoundary() : () => {};
  return { exports, sink, frameBoundary };
}

function describeToken(t) {
  if (/^[0-9a-f]{8}$/.test(t)) return `${t}(${bitsToF32(parseInt(t, 16))})`;
  return t;
}

function describeDiff(expected, actual) {
  const e = expected.split(' ');
  const a = actual.split(' ');
  const parts = [];
  for (let i = 0; i < Math.max(e.length, a.length); ++i) {
    if (e[i] !== a[i]) {
      parts.push(`token ${i}: c ${e[i] === undefined ? '<none>' : describeToken(e[i])} as ${a[i] === undefined ? '<none>' : describeToken(a[i])}`);
    }
  }
  return parts.join('; ');
}

function isNaNToken(t) {
  if (!/^[0-9a-f]{8}$/.test(t)) return false;
  const bits = parseInt(t, 16);
  return (bits & 0x7f800000) === 0x7f800000 && (bits & 0x007fffff) !== 0;
}

// True when the lines differ only where both sides hold some NaN.
function differsOnlyInNaN(expected, actual) {
  const e = expected.split(' ');
  const a = actual.split(' ');
  if (e.length !== a.length) return false;
  for (let i = 0; i < e.length; ++i) {
    if (e[i] !== a[i] && !(isNaNToken(e[i]) && isNaNToken(a[i]))) return false;
  }
  return true;
}

// Compares line by line and prints the first maxShown mismatches, each with
// the context lines before it. nanEqual lets any NaN match any NaN: clang may
// commute an operation whose operands are both NaN, which picks the other
// payload, so with two NaN inputs the payload is not part of the contract.
// Returns { checks, tokens, mismatches, nanOnly }.
export function compareLines(name, expected, actual, { maxShown = 10, context = 2, nanEqual = false } = {}) {
  let mismatches = 0;
  let nanOnly = 0;
  let tokens = 0;
  const n = Math.min(expected.length, actual.length);
  for (let i = 0; i < n; ++i) {
    const e = expected[i];
    const a = actual[i];
    tokens += e.length === 0 ? 0 : e.split(' ').length;
    if (e === a) continue;
    if (nanEqual && differsOnlyInNaN(e, a)) {
      nanOnly += 1;
      continue;
    }
    mismatches += 1;
    if (mismatches <= maxShown) {
      console.log(`${name}: mismatch at line ${i + 1}`);
      for (let k = Math.max(0, i - context); k < i; ++k) console.log(`    ${k + 1}  ${expected[k]}`);
      console.log(`  c ${i + 1}  ${e}`);
      console.log(`  as ${i + 1} ${a}`);
      console.log(`    ${describeDiff(e, a)}`);
    }
  }
  if (expected.length !== actual.length) {
    mismatches += 1;
    console.log(`${name}: line count differs, c ${expected.length} as ${actual.length}`);
    const i = n;
    if (i < expected.length) console.log(`  first missing c line ${i + 1}: ${expected[i]}`);
    if (i < actual.length) console.log(`  first extra as line ${i + 1}: ${actual[i]}`);
  }
  if (mismatches > maxShown) console.log(`${name}: ${mismatches - maxShown} more mismatches not shown`);
  return { checks: n, tokens, mismatches, nanOnly };
}

// The usual comparator: load verify/out/<name>.wasm, let drive(exports, ctx)
// call the driver, compare with verify/vectors/<name>.txt, print a summary and
// exit nonzero on any mismatch.
export async function runModule(name, drive, options = {}) {
  const wasmPath = process.env.WASM ?? join(verifyDir, 'out', `${name}.wasm`);
  const vectorPath = process.env.VECTORS ?? join(verifyDir, 'vectors', `${name}.txt`);
  const expected = readVectors(vectorPath);
  const ctx = loadWasm(wasmPath, options.imports);
  try {
    await drive(ctx.exports, ctx);
  } catch (err) {
    console.log(`${name}: driver trapped after ${ctx.sink.lines.length} lines: ${err.message}`);
    if (ctx.sink.lines.length > 0) console.log(`  last as line: ${ctx.sink.lines[ctx.sink.lines.length - 1]}`);
    const i = ctx.sink.lines.length;
    if (i < expected.length) console.log(`  next c line ${i + 1}: ${expected[i]}`);
    console.log(`RESULT ${name} 0 1`);
    process.exit(1);
  }
  const result = compareLines(name, expected, ctx.sink.lines, options);
  const nanNote = result.nanOnly > 0 ? `, ${result.nanOnly} lines differ only in NaN payloads` : '';
  console.log(`${name}: ${result.checks} lines, ${result.tokens} values, ${result.mismatches} mismatches${nanNote}`);
  console.log(`RESULT ${name} ${result.checks} ${result.mismatches}`);
  process.exit(result.mismatches === 0 ? 0 : 1);
}
