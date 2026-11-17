// SPDX-License-Identifier: MIT
// The SID host's emulated machine (host/), driven through its
// native harness with a PSID built here: the driver calls init and play at the
// rate the header asks for, writes are recorded with their cycles, and the
// header rules from SID_file_format.txt are enforced.
import test, { after } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawnSync } from 'node:child_process';

import { commandExists } from '../tools/lib/commands.mjs';

const HOST_DIR = path.join(import.meta.dirname, '..', 'host');
const HARNESS_SOURCE = path.join(import.meta.dirname, 'sidhost_harness.cpp');

const NOT_CI = ['', '0', 'false'];
const inCi = !NOT_CI.includes(process.env.CI ?? '');
const skip = !commandExists('c++') && !inCi && 'c++ is not on PATH';

// One directory for the built harness and every PSID the tests write, gone when they are done.
const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'sidhost-'));
after(() => fs.rmSync(dir, { recursive: true, force: true }));

let harnessPath;
function harness() {
  if (harnessPath) return harnessPath;
  harnessPath = path.join(dir, 'harness');
  const built = spawnSync('c++', ['-std=gnu++17', '-O2', '-Wall', '-Wextra', '-I', HOST_DIR, '-o', harnessPath,
    HARNESS_SOURCE, path.join(HOST_DIR, 'SidMachine.cpp')], { encoding: 'utf8' });
  assert.equal(built.status, 0, built.stderr);
  return harnessPath;
}

// A PSID v2 whose init stores the song number and sets the volume, and whose
// play counts calls into $D400 and echoes the song into $D401.
function makePsid({ magic = 'PSID', load = 0x1000, loadInHeader = true, speed = 0b10, songs = 2, play = load + 0x10, init = load, kernalExit = false, sid2 = 0 } = {}) {
  const header = Buffer.alloc(0x7C);
  header.write(magic, 0, 'latin1');
  header.writeUInt16BE(sid2 ? 3 : 2, 4);
  if (sid2) header[0x7A] = sid2;             // v3: the second SID's address, middle byte of $Dxx0
  header.writeUInt16BE(0x7C, 6);
  header.writeUInt16BE(loadInHeader ? load : 0, 8);
  header.writeUInt16BE(init, 0x0A);
  header.writeUInt16BE(play, 0x0C);
  header.writeUInt16BE(songs, 0x0E);
  header.writeUInt16BE(1, 0x10);
  header.writeUInt32BE(speed, 0x12);
  header.write('Counter', 0x16, 'latin1');
  header.write('Test', 0x36, 'latin1');
  header.write('2026', 0x56, 'latin1');
  header.writeUInt16BE(0x0004, 0x76); // PAL
  const lo = (a) => a & 0xFF, hi = (a) => a >> 8;
  const song = load + 0x30, count = load + 0x31;
  const code = Buffer.alloc(0x32, 0xEA);
  Buffer.from([
    0x8D, lo(song), hi(song),        // init: STA song
    0xA9, 0x0F, 0x8D, 0x18, 0xD4,    //       LDA #$0F  STA $D418
    0xA9, 0x00, 0x8D, lo(count), hi(count), // LDA #0  STA count
    0x60,                            //       RTS
  ]).copy(code, 0);
  Buffer.from([
    0xEE, lo(count), hi(count),      // play: INC count
    0xAD, lo(count), hi(count), 0x8D, 0x00, 0xD4, // LDA count  STA $D400
    0xAD, lo(song), hi(song), 0x8D, 0x01, 0xD4,   // LDA song   STA $D401
    ...(kernalExit ? [0x4C, 0x31, 0xEA] : [0x60]), //  JMP $EA31, as a ripped IRQ handler does, or RTS
  ]).copy(code, 0x10);
  code[0x30] = 0; code[0x31] = 0;    // song, count
  const prefix = loadInHeader ? Buffer.alloc(0) : Buffer.from([lo(load), hi(load)]);
  const file = path.join(dir, `t${Math.random().toString(36).slice(2)}.sid`);
  fs.writeFileSync(file, Buffer.concat([header, prefix, code]));
  return file;
}

function run(file, args) {
  const ran = spawnSync(harness(), [file, ...args], { encoding: 'utf8' });
  const lines = ran.stdout.split(/\r?\n/).filter(Boolean);
  return { status: ran.status, header: lines[0], rows: lines.filter((l) => !l.startsWith('#')).map((l) => l.split(' ').slice(1).map((h) => parseInt(h, 16))), notes: lines.filter((l) => l.startsWith('#')) };
}

test('the header is parsed and echoed; a VBI song is called once per frame', { skip }, () => {
  harness();
  const got = run(makePsid(), ['-n', '50']);
  assert.equal(got.status, 0);
  assert.equal(got.header, '# PSID v2 load $1000-$1031 init $1000 play $1010 songs 2 start 1 clock PAL speed VBI driver $0400 rows calls');
  assert.equal(got.rows.length, 50);
  got.rows.forEach((regs, i) => {
    assert.equal(regs[0], i + 1, `row ${i} counts the calls`);
    assert.equal(regs[1], 0, 'song 0');
    assert.equal(regs[0x18], 0x0F, 'init set the volume');
  });
  // Rows per frame: exactly one call lands in each frame after init.
  const frames = run(makePsid(), ['-n', '40', '--rows', 'frames']);
  frames.rows.forEach((regs, i) => assert.equal(regs[0], i, `frame ${i}`));
});

test('a CIA-speed song runs from the 60 Hz timer the format description names', { skip }, () => {
  harness();
  const got = run(makePsid(), ['-a', '1', '-n', '50', '--rows', 'frames']);
  assert.match(got.header, / speed CIA /);
  assert.equal(got.rows[49][1], 1, 'song 1');
  // 49 PAL frames = 963,144 cycles; a 0x4026-cycle timer fires 58 times in them.
  assert.ok(got.rows[49][0] >= 57 && got.rows[49][0] <= 59, `calls after 49 frames: ${got.rows[49][0]}`);
});

test('an embedded load address is honored and the driver moves off a page the tune uses', { skip }, () => {
  harness();
  const got = run(makePsid({ load: 0x0400, loadInHeader: false }), ['-n', '3']);
  assert.equal(got.status, 0);
  assert.match(got.header, /load \$0400-\$0431 init \$0400 play \$0410 .* driver \$0500 /);
  assert.deepEqual(got.rows.map((r) => r[0]), [1, 2, 3]);
});

test('a play routine that leaves through the KERNAL exit still counts and unwinds', { skip }, () => {
  harness();
  const got = run(makePsid({ kernalExit: true }), ['-n', '50']);
  assert.equal(got.status, 0);
  assert.equal(got.rows.length, 50, got.notes.join('; '));
  got.rows.forEach((regs, i) => assert.equal(regs[0], i + 1, `row ${i}`));
});

test('the trace lists every SID write with its cycle, in order', { skip }, () => {
  harness();
  const ran = spawnSync(harness(), [makePsid(), '-n', '2', '--trace'], { encoding: 'utf8' });
  const writes = ran.stdout.split(/\r?\n/).filter((l) => l && !l.startsWith('#')).map((l) => l.split(' '));
  assert.ok(writes.length >= 3, ran.stdout);
  assert.deepEqual(writes[0].slice(1), ['D418', '0F'], 'init writes the volume first');
  const cycles = writes.map((w) => Number(w[0]));
  assert.ok(cycles.every((c, i) => i === 0 || c > cycles[i - 1]), 'cycles increase');
  const playWrites = writes.slice(1);
  assert.deepEqual(playWrites.slice(0, 2).map((w) => w[1]), ['D400', 'D401']);
  assert.ok(cycles[1] >= 19656 && cycles[1] < 2 * 19656, `first play write lands in the second frame: ${cycles[1]}`);
});

test('skipping the idle loop moves no SID write by a cycle, on the raster and on the CIA timer', { skip }, () => {
  harness();
  for (const song of ['0', '1']) {
    const trace = (extra) => {
      const out = spawnSync(harness(), [makePsid(), '-a', song, '-n', '50', '--trace', ...extra], { encoding: 'utf8' }).stdout;
      const skipped = Number(/^# (\d+) idle cycles skipped$/m.exec(out)?.[1]);
      return { writes: out.split(/\r?\n/).filter((l) => l && !l.startsWith('#')), skipped };
    };
    const fast = trace([]);
    const slow = trace(['--no-fast-idle']);
    assert.ok(fast.writes.length > 50, `song ${song}: ${fast.writes.length} writes`);
    assert.ok(fast.skipped > 0, `song ${song}: the fast run skipped nothing`);
    assert.equal(slow.skipped, 0, `song ${song}`);
    assert.deepEqual(fast.writes, slow.writes, `song ${song}`);
  }
});

// The --calls listing: F <frame> <startCycle> <calls> <writes>, C <at> <span> <count>, W <offset> <addr> <value>.
function callWrites(out) {
  const got = [];
  let start = 0, at = 0;
  for (const l of out.split(/\r?\n/)) {
    const f = l.split(' ');
    if (f[0] === 'F') start = Number(f[2]);
    else if (f[0] === 'C') at = start + Number(f[1]);
    else if (f[0] === 'W') got.push(`${at + Number(f[1])} ${f[2]} ${f[3]}`);
  }
  return got;
}

test('the bus calls replay every traced write on its cycle, in order, and a long gap starts a new call', { skip }, () => {
  harness();
  for (const song of ['0', '1']) {
    const file = makePsid();
    const trace = spawnSync(harness(), [file, '-a', song, '-n', '50', '--trace'], { encoding: 'utf8' }).stdout
      .split(/\r?\n/).filter((l) => l && !l.startsWith('#'));
    for (const gap of ['0', '7', '8', '64']) {
      const out = spawnSync(harness(), [file, '-a', song, '-n', '50', '--calls', '--max-gap', gap], { encoding: 'utf8' }).stdout;
      assert.deepEqual(callWrites(out), trace, `song ${song} gap ${gap}`);
      // Play stores $D400 and then $D401 8 cycles later (LDA abs, STA abs): one call when the gap allows 8.
      const calls = out.split(/\r?\n/).filter((l) => l.startsWith('C ')).map((l) => l.split(' ').map(Number));
      const play = calls.slice(1);
      assert.ok(play.length > 0, `song ${song} gap ${gap}`);
      if (Number(gap) >= 8) play.forEach((c) => assert.deepEqual([c[2], c[3]], [8, 2], `song ${song} gap ${gap}: ${c}`));
      else play.forEach((c) => assert.deepEqual([c[2], c[3]], [0, 1], `song ${song} gap ${gap}: ${c}`));
      const frames = out.split(/\r?\n/).filter((l) => l.startsWith('F ')).map((l) => l.split(' ').map(Number));
      assert.deepEqual(frames.map((f) => f[1]), frames.map((_, i) => i), 'frames come out in order through both slots');
      assert.ok(frames.every((f, i) => f[2] === 19656 * i), 'each frame starts a PAL frame after the last');
    }
  }
});

test('a call ends at the frame end and at the span cap, and a full slot drops the rest of its frame', { skip }, () => {
  harness();
  const file = makePsid();
  const lines = (args) => spawnSync(harness(), [file, ...args], { encoding: 'utf8' }).stdout.split(/\r?\n/);
  const parse = (out) => out.filter((l) => /^[FC] /.test(l)).map((l) => l.split(' ').map((v, i) => (i ? Number(v) : v)));
  for (const song of ['0', '1']) {
    const trace = lines(['-a', song, '-n', '50', '--trace']).filter((l) => l && !l.startsWith('#'));
    // No gap or span limit: the frame end is the only cut, so each frame is one call that stays inside it.
    const whole = lines(['-a', song, '-n', '50', '--calls', '--max-gap', '1000000', '--max-span', '1000000']);
    assert.deepEqual(callWrites(whole.join('\n')), trace, `song ${song}, frame cuts only`);
    for (const f of parse(whole).filter((r) => r[0] === 'F')) assert.ok(f[3] <= 1, `song ${song}: frame ${f[1]} has ${f[3]} calls`);
    for (const c of parse(whole).filter((r) => r[0] === 'C')) assert.ok(c[1] + c[2] < 19656, `song ${song}: call ${c} crosses the frame end`);
    // Play's two stores are 8 cycles apart: a 7-cycle cap splits them, an 8-cycle cap keeps them together.
    for (const [cap, span, count] of [[7, 0, 1], [8, 8, 2]]) {
      const out = lines(['-a', song, '-n', '50', '--calls', '--max-gap', '1000000', '--max-span', String(cap)]);
      assert.deepEqual(callWrites(out.join('\n')), trace, `song ${song} span ${cap}`);
      const play = parse(out).filter((r) => r[0] === 'C').slice(1);
      play.forEach((c) => assert.deepEqual([c[2], c[3]], [span, count], `song ${song} span ${cap}: ${c}`));
    }
  }
  // One write a slot: every play frame overflows, keeps its first write, and says so.
  const small = lines(['-n', '10', '--calls', '--slot-writes', '1']);
  const frames = parse(small).filter((r) => r[0] === 'F');
  assert.ok(frames.slice(1).every((f) => f[4] === 1 && f[5] === 1), small.join('\n'));
  assert.match(small.join('\n'), /overflowed frames 9\b/);
  assert.deepEqual(small.filter((l) => l.startsWith('W ')).slice(1).map((l) => l.split(' ')[2]), Array(9).fill('D400'));
});

test('a second SID at $DFE0 does not take the driver marker for one of its writes', { skip }, () => {
  harness();
  const got = run(makePsid({ sid2: 0xFE }), ['-n', '3']);
  assert.equal(got.status, 0);
  assert.match(got.header, / rows calls$/);
  assert.deepEqual(got.rows.map((r) => r[0]), [1, 2, 3], got.notes.join('; '));
  const trace = spawnSync(harness(), [makePsid({ sid2: 0xFE }), '-n', '2', '--trace'], { encoding: 'utf8' }).stdout;
  assert.ok(!/ DFFF /.test(trace), 'the marker store is not a recorded write');
});

test('the native start, the prologue a real C64 runs in a capture, gives the same calls as the emulated setup', { skip }, () => {
  harness();
  const file = makePsid();
  for (const song of ['0', '1']) {
    const writes = (extra) => spawnSync(harness(), [file, '-a', song, '-n', '50', '--calls', ...extra], { encoding: 'utf8' }).stdout
      .split(/\r?\n/).filter((l) => l.startsWith('W '));
    const plain = writes([]);
    assert.ok(plain.length > 50, `song ${song}`);
    // The prologue waits for raster line $FA, so a CIA song's timer starts that much
    // later and the native run can end a call or two short; up to there they agree.
    for (const extra of [['--native'], ['--native', '--blank']]) {
      const native = writes(extra);
      assert.ok(native.length >= plain.length - 4, `song ${song} ${extra}: ${native.length} of ${plain.length} writes`);
      assert.deepEqual(native, plain.slice(0, native.length), `song ${song} ${extra}`);
    }
  }
});

// A capture file: 32-byte header (magic, version, flags, song, count, cycles), then
// 8-byte records (cycle, address, value, pad), little-endian.
function compare(sid, cap) {
  const out = spawnSync(harness(), [sid, '--compare', cap], { encoding: 'utf8' }).stdout;
  const m = /^# compare: play calls (\d+) calls (\d+) exact (\d+) writes-differ (\d+) offsets-differ (\d+) call-count-differs (\d+)$/m.exec(out);
  assert.ok(m, out);
  const [playCalls, calls, exact, writes, offsets, shape] = m.slice(1).map(Number);
  return { playCalls, calls, exact, writes, offsets, shape, out };
}

test('a capture compares play call by play call, and a changed value, order or offset is reported', { skip }, () => {
  harness();
  const sid = makePsid();
  for (const song of ['0', '1']) {
    const cap = path.join(dir, `c${song}.cap`);
    const wrote = spawnSync(harness(), [sid, '-a', song, '-n', '100', '--blank', '--write-cap', cap], { encoding: 'utf8' });
    assert.equal(wrote.status, 0, wrote.stderr);
    const bytes = fs.readFileSync(cap);
    assert.equal(bytes.toString('latin1', 0, 7), 'SIDCAP1');
    assert.equal(bytes[11], Number(song));
    assert.equal(bytes[10], 0, 'PAL, screen off');
    const records = (bytes.length - 32) / 8;
    assert.equal(bytes.readUInt32LE(12), records);
    const rec = (i) => ({ cycle: bytes.readUInt32LE(32 + i * 8), addr: bytes.readUInt16LE(36 + i * 8), value: bytes[38 + i * 8] });
    assert.ok([...Array(records).keys()].some((i) => rec(i).addr === 0xDFFF), 'the play markers are in the log');

    const clean = compare(sid, cap);
    assert.ok(clean.playCalls >= 90, clean.out);
    assert.equal(clean.exact, clean.calls, clean.out);
    assert.deepEqual([clean.writes, clean.offsets, clean.shape], [0, 0, 0], clean.out);
    assert.match(clean.out, /largest difference 0 cycles/);

    // The fifth play call's $D400/$D401 pair, 8 cycles apart.
    const pairs = [...Array(records - 1).keys()].filter((i) => rec(i).addr === 0xD400 && rec(i + 1).addr === 0xD401);
    const at = pairs[5];
    const edit = (name, change) => {
      const copy = Buffer.from(bytes);
      change(copy);
      const file = path.join(dir, `c${song}-${name}.cap`);
      fs.writeFileSync(file, copy);
      return compare(sid, file);
    };
    const value = edit('value', (b) => { b[38 + (at + 1) * 8] ^= 0x40; });
    assert.deepEqual([value.writes, value.offsets, value.shape], [1, 0, 0], value.out);
    const order = edit('order', (b) => {
      b.writeUInt16LE(0xD401, 36 + at * 8); b.writeUInt16LE(0xD400, 36 + (at + 1) * 8);
      const v = b[38 + at * 8]; b[38 + at * 8] = b[38 + (at + 1) * 8]; b[38 + (at + 1) * 8] = v;
    });
    assert.deepEqual([order.writes, order.offsets, order.shape], [1, 0, 0], order.out);
    const offset = edit('offset', (b) => { b.writeUInt32LE(rec(at + 1).cycle + 1, 32 + (at + 1) * 8); });
    assert.deepEqual([offset.writes, offset.offsets, offset.shape], [0, 1, 0], offset.out);
    assert.match(offset.out, /^# offsets differ by: 1 1, /m);
    assert.match(offset.out, /^! play call \d+, call 0: offsets differ$/m);
    // $D401 100 cycles late: past the 64-cycle gap, so the captured play call has two calls.
    const split = edit('split', (b) => { b.writeUInt32LE(rec(at + 1).cycle + 100, 32 + (at + 1) * 8); });
    assert.deepEqual([split.writes, split.offsets, split.shape], [0, 0, 1], split.out);
    assert.match(split.out, /^! play call \d+: 2 calls captured, 1 emulated$/m);
  }
});

test('headers the format description says to reject are rejected', { skip }, () => {
  harness();
  const rsid = run(makePsid({ magic: 'RSID' }), ['-n', '1']); // RSID with a header load address, play address and speed bits
  assert.equal(rsid.status, 3);
  assert.match(rsid.notes[0], /header rejected/);
});
