// SPDX-License-Identifier: MIT
// The SID host's emulated machine (Source/Teensy/SIDHost/), driven through its
// native harness with a PSID built here: the driver calls init and play at the
// rate the header asks for, writes are recorded with their cycles, and the
// header rules from SID_file_format.txt are enforced.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawnSync } from 'node:child_process';

import { commandExists } from '../../../tools/lib/toolchain.mjs';

const HOST_DIR = path.join(import.meta.dirname, '..', 'SIDHost');
const HARNESS_SOURCE = path.join(import.meta.dirname, 'sidhost_harness.cpp');

const NOT_CI = ['', '0', 'false'];
const inCi = !NOT_CI.includes(process.env.CI ?? '');
const skip = !commandExists('c++') && !inCi && 'c++ is not on PATH';

let harnessPath, dir;
function harness() {
  if (harnessPath) return harnessPath;
  dir = fs.mkdtempSync(path.join(os.tmpdir(), 'sidhost-'));
  harnessPath = path.join(dir, 'harness');
  const built = spawnSync('c++', ['-std=gnu++17', '-O2', '-Wall', '-Wextra', '-I', HOST_DIR, '-o', harnessPath,
    HARNESS_SOURCE, path.join(HOST_DIR, 'SidMachine.cpp')], { encoding: 'utf8' });
  assert.equal(built.status, 0, built.stderr);
  return harnessPath;
}

// A PSID v2 whose init stores the song number and sets the volume, and whose
// play counts calls into $D400 and echoes the song into $D401.
function makePsid({ magic = 'PSID', load = 0x1000, loadInHeader = true, speed = 0b10, songs = 2, play = load + 0x10, init = load, kernalExit = false } = {}) {
  const header = Buffer.alloc(0x7C);
  header.write(magic, 0, 'latin1');
  header.writeUInt16BE(2, 4);
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
  const file = path.join(dir ?? fs.mkdtempSync(path.join(os.tmpdir(), 'sidhost-')), `t${Math.random().toString(36).slice(2)}.sid`);
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

test('headers the format description says to reject are rejected', { skip }, () => {
  harness();
  const rsid = run(makePsid({ magic: 'RSID' }), ['-n', '1']); // RSID with a header load address, play address and speed bits
  assert.equal(rsid.status, 3);
  assert.match(rsid.notes[0], /header rejected/);
});
