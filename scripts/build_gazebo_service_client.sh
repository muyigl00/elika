#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
mkdir -p .cache/bin
source_file=scripts/gazebo_service_client.cc
binary=.cache/bin/gazebo_service_client
if [[ -x "$binary" && "$binary" -nt "$source_file" ]]; then exit 0; fi
for directory in /opt/ros/jazzy/opt/*/lib/pkgconfig /opt/ros/jazzy/opt/*/share/pkgconfig; do
  if [[ -d "$directory" ]]; then export PKG_CONFIG_PATH="$directory:${PKG_CONFIG_PATH:-}"; fi
done
read -r -a compile_flags <<< "$(pkg-config --cflags gz-transport13 gz-msgs10)"
read -r -a link_flags <<< "$(pkg-config --libs gz-transport13 gz-msgs10)"
temporary="${binary}.$$"
trap 'rm -f -- "$temporary"' EXIT
g++ -O2 -Wall -Wextra "${compile_flags[@]}" "$source_file" -o "$temporary" "${link_flags[@]}"
mv -- "$temporary" "$binary"

