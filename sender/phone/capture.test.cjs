const assert = require("node:assert/strict");
const { encodeFrame } = require("./capture.js");
const buffer = encodeFrame(2, 2, 123, 987654321,
  new Uint8ClampedArray([255,0,0,255, 0,255,0,255, 0,0,255,255, 255,255,255,255]));
const wire = Buffer.from(buffer);
assert.equal(wire.subarray(0,4).toString(), "SLF1");
assert.equal(wire.readUInt32BE(4), 2);
assert.equal(wire.readUInt32BE(8), 2);
assert.equal(wire.readBigUInt64BE(12), 123n);
assert.equal(wire.readBigUInt64BE(20), 987654321n);
assert.deepEqual([...wire.subarray(28)], [76,150,29,255]);
for (const args of [[0,2,0,0,[]], [2,2,-1,0,[]], [2,2,0,NaN,[]],
                    [2,2,0,Number.MAX_SAFE_INTEGER+1,[]], [1920,1920,0,0,[]]]) {
  assert.throws(() => encodeFrame(...args));
}
console.log("capture encoder tests passed");
