// SPDX-License-Identifier: MIT
//
// Runs the SID host's emulated machine (Source/Teensy/SIDHost/) over a SID
// collection and reports what it could not play, and, given a siddump binary,
// where its register rows differ from siddump's. Nothing here needs a board.
//
//   node tools/sidhost/corpus-check.mjs --hvsc <C64Music dir> [--sample 500] [--seconds 10]
//        [--siddump <siddump exe>] [--harness <built harness>] [--filter PSID|RSID] [--seed 1]
//        [--list <file of relative paths>] [--out report.json] [--jobs 8]
//   node tools/sidhost/corpus-check.mjs --hvsc <dir> --siddump <exe> --explain <relative path> [--seconds 10]
//
// Without --siddump it is a smoke pass: header rejected, driver never called,
// slow (play spends frames in delay loops), silent (every row all-zero),
// write-buffer overflow. With --siddump each tune's
// rows are compared field by field with siddump's table (frequency, waveform,
// ADSR, pulse per voice; filter cutoff, routing, type and volume), one row per
// play call on both sides. A tune that only matches with --flat-io (VIC and CIA
// registers reading back their written values, as siddump's flat memory does)
// is reported as reads-timing: a reference limitation, not an emulation defect.
// Likewise shift-1 (the tune's init enables interrupts, so its first play call
// lands before init returns), own-irq (play address 0: siddump calls the
// vector once per frame, whatever rate the tune's own timer runs at) and
// mirror-writes (the tune writes the SID through $D420-$D7FF, which the chip
// answers to and a flat memory keeps apart) and reads-port (the tune reads
// $00/$01, where a C64 holds $2F/$37 and siddump's memory holds zero).
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawnSync, spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const args = process.argv.slice(2);
const option = (name, fallback) => { const i = args.indexOf(name); return i < 0 ? fallback : args[i + 1]; };
const hvsc = option('--hvsc', process.env.HVSC_DIR);
if (!hvsc) { console.error('--hvsc <dir> (or HVSC_DIR) is required'); process.exit(2); }
const sample = Number(option('--sample', 0));
const seconds = Number(option('--seconds', 10));
const siddump = option('--siddump', null);
const filter = option('--filter', null);
const seed = Number(option('--seed', 1));
const listFile = option('--list', null);
const out = option('--out', null);
const jobs = Number(option('--jobs', Math.max(1, Math.min(8, os.cpus().length - 1))));
const explain = option('--explain', null);
// Extra harness options, for experiments: --harness-arg --ntsc --harness-arg --flat-io
const harnessArgs = args.flatMap((a, i) => (a === '--harness-arg' ? [args[i + 1]] : []));
const compiler = process.env.CXX ?? 'c++';

function buildHarness() {
  const given = option('--harness', null);
  if (given) return given;
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'sidhost-'));
  const exe = path.join(dir, process.platform === 'win32' ? 'harness.exe' : 'harness');
  const built = spawnSync(compiler, ['-std=gnu++17', '-O2', '-I', path.join(root, 'Source/Teensy/SIDHost'), '-o', exe,
    path.join(root, 'Source/Teensy/tests/sidhost_harness.cpp'), path.join(root, 'Source/Teensy/SIDHost/SidMachine.cpp')],
    { encoding: 'utf8', windowsHide: true });
  if (built.status !== 0) { console.error(built.stderr); process.exit(1); }
  return exe;
}

function* walk(dir) {
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    const p = path.join(dir, entry.name);
    if (entry.isDirectory()) { if (entry.name !== 'DOCUMENTS' && entry.name !== 'update') yield* walk(p); }
    else if (entry.name.toLowerCase().endsWith('.sid')) yield p;
  }
}

// Deterministic sampling, so a rerun with the same seed sees the same tunes.
function shuffle(list, s) {
  let x = s >>> 0 || 1;
  const rnd = () => { x ^= x << 13; x >>>= 0; x ^= x >>> 17; x ^= x << 5; x >>>= 0; return x / 0x100000000; };
  for (let i = list.length - 1; i > 0; i--) { const j = Math.floor(rnd() * (i + 1)); [list[i], list[j]] = [list[j], list[i]]; }
  return list;
}

function headerOf(file) {
  const fd = fs.openSync(file, 'r'); const b = Buffer.alloc(0x7C); const n = fs.readSync(fd, b, 0, 0x7C, 0); fs.closeSync(fd);
  return { magic: b.toString('latin1', 0, 4), version: b.readUInt16BE(4), play: b.readUInt16BE(0x0C), songs: b.readUInt16BE(0x0E),
           start: b.readUInt16BE(0x10) || 1, flags: n >= 0x78 ? b.readUInt16BE(0x76) : 0 };
}

// Parses the harness's rows into Uint8Array(25) register images, plus its notes.
function parseHarness(text) {
  const rows = [], notes = [];
  let header = '';
  for (const line of text.split(/\r?\n/)) {
    if (!line) continue;
    if (line.startsWith('# ')) { if (!header) header = line; else if (!line.startsWith('# name')) notes.push(line.slice(2)); continue; }
    rows.push(Uint8Array.from(line.split(' ').slice(1, 26), (h) => parseInt(h, 16)));
  }
  return { header, rows, notes };
}

// Parses siddump's table into the fields the harness rows carry, undoing its
// "unchanged" dots. The note column between frequency and waveform varies in
// width, so fields are taken from each end of a voice column.
const FILTER_NAMES = ['Off', 'Low', 'Bnd', 'L+B', 'Hi', 'L+H', 'B+H', 'LBH'];
function parseSiddump(text) {
  const rows = [];
  let prev = null;
  for (const line of text.split(/\r?\n/)) {
    if (!/^\|\s*\d/.test(line)) continue;
    const cols = line.split('|').slice(2, 6).map((c) => c.trim().split(/\s+/));
    if (cols.length < 4) continue;
    const row = { voices: [], cutoff: 0, ctrl: 0, type: 0, vol: 0 };
    for (let v = 0; v < 3; v++) {
      const t = cols[v], n = t.length;
      const p = prev ? prev.voices[v] : { freq: 0, wave: 0, adsr: 0, pulse: 0 };
      row.voices.push({
        freq: t[0].startsWith('.') ? p.freq : parseInt(t[0], 16),
        wave: t[n - 3].startsWith('.') ? p.wave : parseInt(t[n - 3], 16),
        adsr: t[n - 2].startsWith('.') ? p.adsr : parseInt(t[n - 2], 16),
        pulse: t[n - 1].startsWith('.') ? p.pulse : parseInt(t[n - 1], 16),
      });
    }
    const f = cols[3], fp = prev ?? { cutoff: 0, ctrl: 0, type: 0, vol: 0 };
    row.cutoff = f[0].startsWith('.') ? fp.cutoff : parseInt(f[0], 16);
    row.ctrl = f[1].startsWith('.') ? fp.ctrl : parseInt(f[1], 16);
    const typeTok = f.slice(2, f.length - 1).join(' ');
    row.type = typeTok.startsWith('.') ? fp.type : FILTER_NAMES.indexOf(typeTok);
    row.vol = f[f.length - 1].startsWith('.') ? fp.vol : parseInt(f[f.length - 1], 16);
    rows.push(row);
    prev = row;
  }
  return rows;
}

// The same fields from a register image, with siddump's own cutoff arithmetic.
function fieldsOf(regs) {
  const voices = [];
  for (let v = 0; v < 3; v++) {
    const b = 7 * v;
    voices.push({ freq: regs[b] | (regs[b + 1] << 8), pulse: (regs[b + 2] | (regs[b + 3] << 8)) & 0xFFF, wave: regs[b + 4], adsr: (regs[b + 5] << 8) | regs[b + 6] });
  }
  return { voices, cutoff: ((regs[0x15] << 5) | (regs[0x16] << 8)) & 0xFFFF, ctrl: regs[0x17], type: (regs[0x18] >> 4) & 7, vol: regs[0x18] & 0xF };
}

function compare(mine, theirs) {
  const n = Math.min(mine.length, theirs.length);
  let first = -1, bad = 0;
  for (let i = 0; i < n; i++) {
    const a = fieldsOf(mine[i]), b = theirs[i];
    let same = a.cutoff === b.cutoff && a.ctrl === b.ctrl && a.type === b.type && a.vol === b.vol;
    for (let v = 0; v < 3 && same; v++) {
      const x = a.voices[v], y = b.voices[v];
      same = x.freq === y.freq && x.wave === y.wave && x.adsr === y.adsr && x.pulse === y.pulse;
    }
    if (!same) { bad++; if (first < 0) first = i; }
  }
  return { compared: n, bad, first, lengthDiff: mine.length - theirs.length };
}

function runAsync(exe, argv) {
  return new Promise((resolve) => {
    const child = spawn(exe, argv, { windowsHide: true });
    let stdout = '', stderr = '';
    child.stdout.on('data', (d) => { stdout += d; });
    child.stderr.on('data', (d) => { stderr += d; });
    child.on('close', (status) => resolve({ status, stdout, stderr }));
    child.on('error', (error) => resolve({ status: -1, stdout, stderr: String(error) }));
  });
}

async function checkOne(harness, file) {
  const rel = path.relative(hvsc, file).split(path.sep).join('/');
  const h = headerOf(file);
  const song = h.start - 1;
  const result = { file: rel, magic: h.magic, play: h.play, status: 'ok' };
  const mine = await runAsync(harness, [file, '-a', String(song), '-t', String(seconds), ...harnessArgs]);
  if (mine.status === 3) { result.status = 'unsupported'; result.note = mine.stdout.trim(); return result; }
  if (mine.status !== 0) { result.status = 'crash'; result.note = (mine.stderr || `exit ${mine.status}`).trim().slice(0, 200); return result; }
  const parsed = parseHarness(mine.stdout);
  result.rows = parsed.rows.length;
  if (parsed.notes.some((n) => n.includes('play was not called'))) result.status = 'no-play';
  else if (parsed.notes.some((n) => n.startsWith('slow:'))) result.status = 'slow';
  else if (parsed.notes.some((n) => n.includes('overflow'))) result.status = 'overflow';
  else if (parsed.rows.every((r) => r.every((b) => b === 0))) result.status = 'silent';
  if (siddump && result.status === 'ok') {
    const theirs = await runAsync(siddump, [file, `-a${song}`, `-t${seconds}`]);
    const rows = parseSiddump(theirs.stdout);
    if (!rows.length) {
      result.status = 'siddump-failed';
      result.note = theirs.stdout.split(/\r?\n/).find((l) => /Error|Warning/.test(l)) ?? '';
      return result;
    }
    const c = compare(parsed.rows, rows);
    Object.assign(result, { compared: c.compared, bad: c.bad, first: c.first, lengthDiff: c.lengthDiff });
    if (c.bad) {
      result.status = c.bad * 20 < c.compared ? 'near' : 'mismatch';
      // siddump has no clock: VIC and CIA registers read back what was written. A
      // tune that matches once the emulation answers the same way differs only there.
      const flat = parseHarness((await runAsync(harness, [file, '-a', String(song), '-t', String(seconds), '--flat-io'])).stdout);
      const cf = compare(flat.rows, rows);
      // A tune whose init enables interrupts gets its first play call before init has
      // finished, which a reference calling init to completion cannot show: the rows
      // then line up one later.
      const cs = compare(parsed.rows.slice(1), rows);
      if (!cf.bad && cf.compared === c.compared) result.status = 'reads-timing';
      else if (!cs.bad && cs.compared === c.compared - 1) result.status = 'shift-1';
      else if (h.play === 0) result.status = 'own-irq';  // siddump calls the vector once per frame whatever drives it
      else if (parsed.notes.some((n) => n.includes('mirror addresses'))) result.status = 'mirror-writes';  // the real SID hears them; siddump's memory does not
      else if (parsed.notes.some((n) => n.includes('reads of $00/$01'))) result.status = 'reads-port';  // $00 is the DDR ($2F after boot), not the zero siddump holds
      else if (cf.bad < c.bad) result.flatBad = cf.bad;
    }
  }
  return result;
}

// One tune, both tables at the first rows that differ: what to look at.
const fmt = (f) => `${f.voices.map((v) => `${v.freq.toString(16).padStart(4, '0')} ${v.wave.toString(16).padStart(2, '0')} ${v.adsr.toString(16).padStart(4, '0')} ${v.pulse.toString(16).padStart(3, '0')}`).join(' | ')} | ${f.cutoff.toString(16).padStart(4, '0')} ${f.ctrl.toString(16).padStart(2, '0')} ${FILTER_NAMES[f.type] ?? '?'} ${f.vol.toString(16)}`;
async function explainOne(harness, rel) {
  const file = path.join(hvsc, rel);
  const h = headerOf(file), song = h.start - 1;
  const mine = await runAsync(harness, [file, '-a', String(song), '-t', String(seconds), ...harnessArgs]);
  const parsed = parseHarness(mine.stdout);
  const theirs = parseSiddump((await runAsync(siddump, [file, `-a${song}`, `-t${seconds}`])).stdout);
  console.log(parsed.header);
  for (const n of parsed.notes) console.log(`# ${n}`);
  const c = compare(parsed.rows, theirs);
  console.log(`rows: mine ${parsed.rows.length}, siddump ${theirs.length}; ${c.bad}/${c.compared} differ, first at ${c.first}`);
  if (c.first < 0) return;
  console.log('       voice 1 (freq wf adsr pul)| voice 2            | voice 3            | cutoff ctrl type vol');
  for (let i = Math.max(0, c.first - 1); i < Math.min(c.first + 4, c.compared); i++) {
    console.log(`${String(i).padStart(4)} me ${fmt(fieldsOf(parsed.rows[i]))}`);
    console.log(`     sd ${fmt(theirs[i])}`);
  }
}

async function main() {
  const harness = buildHarness();
  if (explain) { await explainOne(harness, explain); return; }
  let files = [];
  if (listFile) files = fs.readFileSync(listFile, 'utf8').split(/\r?\n/).filter(Boolean).map((r) => path.join(hvsc, r));
  else {
    for (const f of walk(hvsc)) {
      if (filter && headerOf(f).magic !== filter) continue;
      files.push(f);
    }
    files.sort();
    if (sample) files = shuffle(files, seed).slice(0, sample).sort();
  }
  console.error(`${files.length} files, ${seconds} s each, ${jobs} jobs${siddump ? ', comparing with siddump' : ''}`);
  const results = [];
  let next = 0, done = 0;
  const started = Date.now();
  await Promise.all(Array.from({ length: jobs }, async () => {
    while (next < files.length) {
      const file = files[next++];
      results.push(await checkOne(harness, file));
      if (++done % 200 === 0) console.error(`${done}/${files.length} (${((Date.now() - started) / 1000).toFixed(0)} s)`);
    }
  }));
  results.sort((a, b) => a.file.localeCompare(b.file));
  const counts = {};
  for (const r of results) counts[r.status] = (counts[r.status] ?? 0) + 1;
  console.log(`checked ${results.length} in ${((Date.now() - started) / 1000).toFixed(0)} s: ${Object.entries(counts).map(([k, v]) => `${k} ${v}`).join(', ')}`);
  for (const status of ['crash', 'unsupported', 'no-play', 'slow', 'silent', 'overflow', 'siddump-failed', 'mismatch', 'near', 'reads-timing', 'shift-1', 'own-irq', 'mirror-writes', 'reads-port']) {
    const list = results.filter((r) => r.status === status);
    if (!list.length) continue;
    console.log(`\n${status} (${list.length}):`);
    for (const r of list.slice(0, 40)) {
      const detail = r.bad !== undefined ? ` first diff row ${r.first}, ${r.bad}/${r.compared} rows differ, length ${r.lengthDiff >= 0 ? '+' : ''}${r.lengthDiff}${r.flatBad !== undefined ? `, ${r.flatBad} with flat I/O` : ''}` : r.note ? ` ${r.note}` : '';
      console.log(`  ${r.file} [${r.magic}${r.play === 0 ? ', play=0' : ''}]${detail}`);
    }
    if (list.length > 40) console.log(`  ... and ${list.length - 40} more`);
  }
  if (out) fs.writeFileSync(out, JSON.stringify({ hvsc, seconds, seed, sample, siddump: !!siddump, counts, results }, null, 1));
}

main();
