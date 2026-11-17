// SPDX-License-Identifier: MIT
//
// Assembles the C64 cartridge stub and writes host/SIDHostStub.bin.h, which the host build
// embeds. The header is committed, so this is only needed after a change to the stub.
//
//   TR_CORE=<TeensyROM checkout> node tools/build-stub.mjs
//
// The assembler and the header writer are the core's, so the output matches what the core's
// build-c64 wrote when the stub lived there.
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { coreRoot } from './lib/core.mjs';

const repo = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const core = coreRoot();
const lib = (name) => import(pathToFileURL(path.join(core, 'tools', 'lib', name)).href);
const { resolveAcme } = await lib('c64-toolchain.mjs');
const { binToHeader } = await lib('bin2header.mjs');

const stubDir = path.join(repo, 'stub');
const buildDir = path.join(stubDir, 'build');
fs.mkdirSync(buildDir, { recursive: true });
const output = path.join(buildDir, 'SIDHostStub.bin');

// ACME rejects any option after a source file, so the source stays last.
const acme = resolveAcme(path.join(core, 'tools', '.cache'));
const run = spawnSync(acme, [
  '-r', path.join(buildDir, 'BuildReport'),
  '--vicelabels', path.join(buildDir, 'Labels'),
  '--msvc', '--format', 'plain', '-v1',
  '--outfile', output,
  'source/SIDHostStub.asm',
], { cwd: stubDir, stdio: 'inherit' });
if (run.status !== 0) process.exit(run.status ?? 1);

const bytes = fs.readFileSync(output);
const dest = path.join(repo, 'host', 'SIDHostStub.bin.h');
fs.writeFileSync(dest, binToHeader(bytes, { name: 'SIDHostStub.bin', typemod: 'PROGMEM ' }));
console.log(`SIDHostStub.bin ${bytes.length} bytes -> ${path.relative(repo, dest)}`);
