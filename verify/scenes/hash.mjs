// Prints bench/as/main.as's state hash of a native recording after a given number of steps:
//   node verify/scenes/hash.mjs verify/scenes/out/pyramid_scalar_w1.bin 200
import fs from 'fs';
const [file, steps] = [process.argv[2], parseInt(process.argv[3])];
const buf = fs.readFileSync(file);
const w = new Uint32Array(buf.buffer, buf.byteOffset, buf.length >> 2);
const counters = w[14], moverWords = w[15];
let i = 16;
for (;;) {
	const step = w[i], n = w[i + 1];
	i += 2 + counters;
	if (step + 1 === steps) {
		let h = 2166136261;
		for (let b = 0; b < n; ++b) {
			const r = i + b * 17 + 4;
			for (let k = 0; k < 13; ++k) {
				const bits = w[r + k];
				for (let s = 0; s < 4; ++s) {
					h = Math.imul(h ^ ((bits >>> (8 * s)) & 0xff), 16777619) >>> 0;
				}
			}
		}
		console.log(`HASH steps=${steps} bodies=${n} hash=${h}`);
		break;
	}
	i += n * 17 + moverWords;
}
