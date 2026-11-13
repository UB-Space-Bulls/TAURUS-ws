#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
case "$(uname -m)" in
  x86_64) arch=x86_64;; aarch64|arm64) arch=aarch64;;
  *) echo 'Only Linux x86_64/aarch64 are supported' >&2; exit 2;;
esac
mkdir -p build
g++ -std=c++17 -O2 -Wall -Wextra -isystem vendor/include src/sdk_reader.cpp src/sdk_serial_drain.cpp "vendor/lib/$arch/libunitree_lidar_sdk.a" -pthread -Wl,--wrap=_ZN6serial6Serial12waitReadableEv -o build/l1_sdk_reader
g++ -std=c++17 -O2 -Wall -Wextra -isystem vendor/include src/wire_reader.cpp -o build/l1_wire_reader
python3 -m unittest discover -s tests -v
echo 'Built: build/l1_sdk_reader and build/l1_wire_reader'
