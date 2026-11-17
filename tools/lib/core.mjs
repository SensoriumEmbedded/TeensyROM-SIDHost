// SPDX-License-Identifier: MIT
//
// Finds the TeensyROM checkout this host builds against. The host overlays its sketch onto the
// core's minimal sketch and includes the core's Common/ headers, so a build needs that tree.
// Set TR_CORE to its root.
import fs from 'node:fs';
import path from 'node:path';

export function coreRoot() {
  const given = process.env.TR_CORE;
  if (!given) throw new Error('Set TR_CORE to the TeensyROM checkout to build against (see CORE_PIN for the commit this was verified on).');
  const root = path.resolve(given);
  if (!fs.existsSync(path.join(root, 'tools', 'build-firmware.mjs'))) {
    throw new Error(`TR_CORE is not a TeensyROM checkout (no tools/build-firmware.mjs): ${root}`);
  }
  return root;
}
