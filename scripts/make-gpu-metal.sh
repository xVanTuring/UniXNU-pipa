#!/bin/bash
# Build the Metal plugin in out/pipa/tree (the Rust bridge with fat LTO: a draw 72 -> 63 us on the tablet, 2026-10-25; LTO=0 for a quick
# build), its owner trust bundle and the publish GPU addon in out/pipa/gpu-metal (the previous addon kept as gpu-addon-clients.prev.bin),
# and keep the plugin's bundle and dSYM and Turnip's unstripped dylib in $KEEP/bin<tag> for decode-render-profile.py.
#   make-gpu-metal.sh TAG     (then: PIPA_GPU_USERSPACE=.../out/pipa/gpu-metal scripts/kernel-round.sh ...)
# Turnip itself is rebuilt with ninja in the tree first when its source changed (the Mesa tree must equal the pin plus the recorded
# patches/mesa of UniXNU for build-turnip-darwin.py; ninja -C build/mesa-tu-ios works on the configured directory either way).
set -eu
tag=$1
keep=${KEEP:-/Users/xvan/agents-home/UniXNU/out/pipa/profile-binaries}
mkdir -p "$keep"
cd /Users/xvan/agents-home/UniXNU/out/pipa/tree
env NMB_FAT_LTO="${LTO:-1}" python3 scripts/build-adreno-metal-plugin.py > "$keep/plugin-$tag.log" 2>&1 ||
  { tail -30 "$keep/plugin-$tag.log"; exit 1; }
tail -1 "$keep/plugin-$tag.log"
python3 scripts/make-owner-trust-bundle.py | tail -1
cp build/adreno-metal-plugin/owner-trust.bundle ../gpu-metal/
mv ../gpu-metal/gpu-addon-clients.bin ../gpu-metal/gpu-addon-clients.prev.bin
python3 scripts/make-gpu-addon.py --publish --clients iconservicesagent,Preferences ../gpu-metal/gpu-addon-clients.bin | tail -1
rm -rf "${keep:?}/bin${tag:?}"
mkdir -p "$keep/bin$tag"
cp -R build/adreno-metal-plugin/NativeAdrenoMetal.bundle build/adreno-metal-plugin/NativeAdrenoMetal.dSYM "$keep/bin$tag/"
cp build/mesa-tu-ios/src/freedreno/vulkan/libvulkan_freedreno.dylib "$keep/bin$tag/libvulkan_freedreno.full.dylib"
echo "kept $keep/bin$tag"
