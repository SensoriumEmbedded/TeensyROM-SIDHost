// SPDX-License-Identifier: MIT
//
// Builds the SID host extension image and packages it as a .TRH, with the core's build:
//
//   TR_CORE=<TeensyROM checkout> node tools/build-host.mjs [--out <dir>] [--tag <name>]
//
// Any further arguments go to the core's tools/build-firmware.mjs unchanged. On Windows the
// toolchain refuses long paths, so build with --out under a short folder such as C:/tr-build/.
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { coreRoot } from './lib/core.mjs';

const repo = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const core = coreRoot();
const run = spawnSync(process.execPath, [
  path.join(core, 'tools', 'build-firmware.mjs'),
  '--target', 'tr-plus',
  '--host-sketch', path.join(repo, 'host'),
  '--host-code-kib', '96',
  ...process.argv.slice(2),
], { cwd: core, stdio: 'inherit' });
process.exit(run.status ?? 1);
