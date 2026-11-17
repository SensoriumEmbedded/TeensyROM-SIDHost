// SPDX-License-Identifier: MIT
import { spawnSync } from 'node:child_process';

export function commandExists(exe) {
  const probe = process.platform === 'win32' ? spawnSync('where', [exe]) : spawnSync('which', [exe]);
  return probe.status === 0;
}
