#!/bin/bash

pip3 install lief==0.13.2

python3 - $@ <<EOF
import sys

import lief

def main():
  if len(sys.argv) != 2:
    print("Usage: make_symbolic.py <path_to_libkortex_driver.so>")
    return 1

  path = sys.argv[1]

  lib = lief.parse(path)
  lib.add(lief.ELF.DynamicEntry(lief.ELF.DYNAMIC_TAGS.SYMBOLIC, 0))
  flags = lib.get(lief.ELF.DYNAMIC_TAGS.FLAGS)
  if flags is None:
      flags = lief.ELF.DynamicEntryFlags(lief.ELF.DYNAMIC_TAGS.FLAGS, 0)
  flags.add(lief.ELF.DYNAMIC_FLAGS.SYMBOLIC)
  lib.add(flags)
  lib.write(path)

  return 0

if __name__ == "__main__":
  sys.exit(main())
EOF
