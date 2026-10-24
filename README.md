# UniXNU-pipa

> **Work in progress.** On the tablet (2026-10-23): the firmware reaches the UniXNU menu, and XNU starts and stops where it needs storage.

Device repository for the Xiaomi Pad 6 (pipa, SM8250), laid out like [UniXNU-sheng](https://github.com/UniXNU/UniXNU-sheng). Firmware: [uefi-pipa](https://github.com/xVanTuring/uefi-pipa), branch `pipa`, a fork of [uefi-sheng](https://github.com/UniXNU/uefi-sheng) whose upstream Mu-Silicium tree already carries `pipaPkg`.

**State on the tablet (2026-10-23):** uefi-pipa 1f6a1520 reaches the UniXNU menu (without a payload it reports that it found no
operating system). With the payload in the boot image (204 MB, accepted by `fastboot boot`), DarwinBoot hands over and XNU stops at
`Failed to mount root device @IOKitBSDInit.cpp:984`, as expected without a storage driver. XNU has no log route yet; the firmware
writes its log into the stock ramoops area (see uefi-pipa). The static checks pass (see [Checks](#checks)); every hardware value
names its public source.

## What is here

| File | Contents |
| --- | --- |
| `board.toml` | Board, IPSW, firmware, payload and ADT settings |
| `configs/pipa_defconfig` | Kernel configuration of the first image |
| `scripts/build-arm-driver.py` | The generic driver archive builder, copied unchanged from UniXNU-sheng |
| `scripts/build-platform-power.py`, `package/ShengPlatformPower.cpp` | The original-PMGR bridge of UniXNU-sheng, which accesses no hardware; the builder takes this board's previous kernel as its reference |
| `scripts/make-pipa-adt.py` | Apple device tree for SM8250: GICv3 0x17a00000/0x17a60000, stride 0x20000, Cortex-A55/A77 cores, 19.2 MHz timebase |
| `scripts/check-pipa.py` | Firmware preflight: UniXNU integration, FD and stack against the memory map, vendor inputs, splash region |
| `scripts/patch-xnu-entry-probe.py`, `scripts/xnu-panic-screen.c` | Copied unchanged from UniXNU-sheng; the payload uses them for the on-screen panic text (there is no UART) |

`uefi-pipa` changes on top of uefi-sheng:

| File | Change |
| --- | --- |
| `Platforms/Xiaomi/pipaPkg/pipa.dsc` | UniXNU, UniXNUDxe, the ramdisk block device driver, DarwinBoot, UniXNULib and FdtLib |
| `Platforms/Xiaomi/pipaPkg/pipa.fdf` | UniXNU and UniXNUDxe in the firmware volume |
| `Platforms/Xiaomi/pipaPkg/Include/DXE.inc` | `ShengBootMemoryDxe` (publishes the boot image ramdisk, the payload volume, as a block device; every range comes from the device tree, so the shengPkg driver serves pipa unchanged) and DarwinBoot |
| `Resources/Configs/pipa.toml` | The boot image base, offsets and page size written out (the mkbootimg defaults upstream packs pipa with) |

The upstream vendor driver set stays as it is; upstream Mu-Silicium lists display, internal storage and side buttons as working on pipa with it.

## Host tools

The unixnu driver needs Python 3.11 or newer and the tools of the [requirements page](https://unixnu.com/guide/requirements). On this Mac these are missing:

```sh
brew install llvm lld cmake ninja mtools dtc
```

Homebrew ships `lld-link` in `lld`, separate from `llvm`; the CLANGPDB tool chain needs both. Python 3.11 comes from pyenv: `pyenv local 3.11.15` in the UniXNU checkout covers the driver and every script it starts there.

## Stage 1 to 3: firmware, menu and payload volume

```sh
export UNIXNU_UEFI=$PWD/uefi-pipa      # the preflight reads the checkout from here
cd UniXNU                              # cloned beside this repository
git submodule update --init --recursive

./unixnu fetch -D ../UniXNU-pipa
./unixnu build package -D ../UniXNU-pipa
./unixnu build firmware -D ../UniXNU-pipa
./unixnu bootimg -D ../UniXNU-pipa --dtb "$UNIXNU_UEFI/Resources/DTBs/pipa.dtb" --ramdisk none

fastboot boot out/pipa/boot/boot.img
```

`fastboot boot` writes nothing to the tablet. Expected: the Mu-Silicium boot log, then the UniXNU menu with its built-in configuration. A firmware stall leaves the tablet in the bootloader; start the same image again.

## Stage 4 to 7: kernel, root, iPadOS and the lock screen

State on 2026-10-24: iPadOS 17.7.11 runs to the home screen on all eight cores at their top clocks, from the UFS root, with the display on the firmware frame buffer (QuartzCore's CPU renderer, slow at 1800x2880). Touch (swipe to unlock) and the power and volume keys (lock, wake) work. No GPU, audio or network yet.

One round is one command, with no keys: `scripts/kernel-round.sh [--no-build | --build-only] [--twice] [--arg NAME=VALUE]... [TAIL_LINES]` builds the kernel, payload and boot image against the local IPSW, `fastboot boot`s it, waits until the kernel restarts the tablet into fastboot by itself (a warm reset that keeps the ramoops log: 10 s after a panic, after the third SpringBoard/backboardd crash, or 240 s after boot), reads the log back and prints the kernel's own lines (display, cores, CPU levels, crashes) and the end. `--twice` after a change of `pipa_defconfig`; `--arg sheng-data-reset=1` once after a change that MobileGestalt caches in the Data overlay (the UI scale).

What makes it up (see `configs/pipa_defconfig` and `board.toml` for every switch and argument):

- `package/BootRamoops.c`: the kernel log in the stock ramoops console zone (stages, process events, exits and crash lines, the message buffer every 100 ms, display and core counters every 10 s), the restart into fastboot (uefi-pipa's RamoopsLogLib sequence), the panic hook the panic screen calls, and `pipa-cpufreq=1` (the EPSS domains to their top levels).
- `drivers/NativeUfs` (branch `pipa`): the SM8250 host, keeping UEFI's SMMU pass-through for streams 0xe0/0x4e0; `native-ufs-probe=16` surveys it without the root.
- userdata: UniXNU-tb128fu's root image (`out/tb128fu/userdata/userdata-root.sparse.img`, the same IPSW and owner key), flashed with `fastboot flash userdata`; the owner root chain, Data seed and trust caches as on UniXNU-tb128fu.
- The display stack of UniXNU-tb128fu without its GPU and touch probes (`drivers/NativeMobileDisplay` branch `pipa`, `package/LegacyDisplayProbe.cpp` and the key store and HID probes), at UI scale 2 (900x1440 points).
- `drivers/ArmCpu` (branch `pipa`): all eight cores, IPIs as SPIs at the distributor's top eight INTIDs (952..959).
- `package/BootFaultProbe.c` and `BootWatchdogPolicy.c` of UniXNU-tb128fu: crash records and the launchd job policy.
- Touch (runs on the tablet since 2026-10-24: axes and range right, about 120 reports a second while a finger moves): `package/TouchProbe.cpp` (`pipa-touch=<stage>`, 5 in `board.toml`) drives the Novatek NT36532 through UniXNU's `drivers/NativeGeniSpi` and `drivers/NativeNovatekTouch` (branch `pipa`: the point reports of this firmware). Stages: 1 a read-only snapshot of clocks and pins, 2 the QUP wrapper 0 clocks and the SPI pins, 3 the GENI SPI engine, a loopback test and the chip id, 4 the firmware download and its info block, 5 the report loop on the attention line (TLMM 39, latched without an interrupt), publishing to the HID input. The board facts and their sources are in `package/TouchBoard.h` (host test `package/TouchBoardTests.c`). The firmware is `novatek_nt36532_m82_fw_tm.bin` from the stock kernel's built-in firmware table (the panel's maker 0x36 pairs with the `_tm` image), kept in `.cache/firmware/touch` outside Git. The kernel log shows the touch rows every 10 s and each finished stroke with its start and end points, for checking the axes; The firmware reports 16-bit coordinates in tenths of a pixel (its info block: 18000 x 28800, Android's range 0..17999 x 0..28799); the driver scales them by that range.
- Keys (run on the tablet since 2026-10-24): `package/Buttons.c` (`pipa-buttons=1`) reads the PM8150 PON's status (power, volume down) and PM8150 GPIO6 (volume up, active low) over SPMI every 10 ms, read only, and passes the pressed set to the HID input (power, volume up and down of its Consumer report). Its first log line shows how the firmware left GPIO6 (an input with a pull-up is mode 0, pull 0, enable 0x80); every change is logged. Decoding and debounce: `package/ButtonsCore.h`, host test `package/ButtonsTests.c`.
- GPU power and clocks (run on the tablet since 2026-10-24): `package/GpuProbe.cpp` (`pipa-gpu=<mode>`, 6 in `board.toml`) after UniXNU-sheng's GPU power probe, with the SM8250 facts in `package/GpuPowerBoard.h` (host test `package/GpuPowerBoardTests.c`). Mode 6 powers the CX domain (it was collapsed), turns on the GPU SMMU's clocks and reads its global space (translation on, no stream mapped: the firmware leaves the GPU alone), reads the AOP command DB at 0x80860000 (gfx, mx and cx level lists; SH0 0x50004, MC0 0x50000, ACV 0x5007c, CN0 0x50074) and brings up the GMU clocks at 200 MHz from GPLL0's divided output; the GMU stays in reset with GX off, ready for a cold start. The rows go to the kernel log as `gpu:` lines.

Inputs kept outside Git in UniXNU's `out/pipa/tree` (copy them again after a tree is wiped): `build/storage-abi/interface-audit.json` and `build/verified-root-os/trusted-key.bin` from `out/tb128fu/tree`, and an empty `build/xnu/ios-obj/EXPORT_HDRS/libsa`.

The first kernel (2026-10-23, before storage) is described below.

`configs/pipa_defconfig` then built the generic kernel path only: CPU, GIC, timer, the device-tree platform expert, the source corecrypto and pthread providers and NativeAES.

A first kernel build from a clean tree needs the bootstrap pass of [UniXNU#5](https://github.com/UniXNU/UniXNU/pull/5) ([UniXNU#4](https://github.com/UniXNU/UniXNU/issues/4)), which UniXNU's main has.

```sh
export UNIXNU_IPSW=$PWD/out/ipsw/iPad_Pro_HFR_17.7.11_21H461_Restore.ipsw   # a local copy of [ipsw] url
./unixnu defconfig pipa_defconfig -D ../UniXNU-pipa
./unixnu ipsw fetch -D ../UniXNU-pipa
./unixnu build kernel -D ../UniXNU-pipa

./unixnu build payload -D ../UniXNU-pipa --unique-chip-id <16 hex digits>
./unixnu bootimg -D ../UniXNU-pipa --dtb "$UNIXNU_UEFI/Resources/DTBs/pipa.dtb"
fastboot boot out/pipa/boot/boot.img
```

The platform seed is any nonzero 64-bit value kept for this tablet, for example from `openssl rand -hex 8`; `.cache/platform-seed` holds the one of this checkout.

On a new tree the PMGR bridge has no reference kernel yet: the first `build kernel` links a no-op in its place and says so, and the second links the bridge.

Expected: DarwinBoot hands over, the entry probe bars (green, blue, magenta) appear at the top of the screen, and the panic screen shows `Failed to mount root device`, as there is no storage driver yet. On the tablet (2026-10-23, the payload of 2026-10-21): the green and blue bars, then `Failed to mount root device @IOKitBSDInit.cpp:984` with the TCR, SCTLR, page and free counts below it. XNU's video console does not draw the kernel log despite `-v`. The same kernel configuration reaches that point on the QEMU board of [UniXNU-qemu](https://github.com/xVanTuring/UniXNU-qemu).

## Values taken from the tablet

Read under Android 14 (HyperOS 2.0.20) with `android-devinfo` on 2026-10-21; the firmware partitions were not copied yet.

| Value | Read | In this repository |
| --- | --- | --- |
| DRAM banks | 0x80000000+0x37700000, 0xc0000000+0x1c0000000, 0x800000000+0x200000000 (15.87 GiB): the added 8 GiB is a bank of its own at 32 GiB | `dram_size = 0x980000000`, base to the end of the last bank |
| Free memory for DarwinBoot's 4 GiB | 0xffc20000..0x27fffffff (6 GiB) in the second bank, 8 GiB in the third | `owned_memory_size = 0x100000000` |
| userdata | sda34, LBA 0x293000..0x1d387fa, 4096-byte blocks (106.6 GiB); the GPT agrees with sysfs | `[userdata]`; `unixnu build userdata --dry-run` passes |
| Storage | SK hynix HN8T05DEHKX073 (UFS 3.1), ufshc 0x1d84000, 6 LUNs | for NativeUfs |
| Boot image header of the stock `boot` | header v3, page 4096 (with `vendor_boot`, header v3); slot `_b` active | the firmware image uses header v1, page 2048, as Mu-Silicium packs pipa |
| CPUs, GIC | MPIDR 0x000..0x700 (four Cortex-A55, four Cortex-A77), GICv3 0x17a00000/0x17a60000 | as in `make-pipa-adt.py` |
| Touch | Novatek `NVT-ts-spi` on the GENI SPI at 0x990000, 9.6 MHz, interrupt TLMM 39; Android reports 0..17999 x 0..28799 | for the touch driver |
| Panel | `mdss_dsi_m82_36_02_0a_dual_dphy_video` (dual DSI, video mode), 1800x2880, 120 Hz; `cont_splash_region` 0x9c000000+0x2300000 | the frame buffer of SimpleFbDxe |
| GPU | Adreno 650v3, chip ID 0x06050002 | |

## Known risks

- **Firmware volume size.** The vendor drivers are 4.8 MiB before compression and the FD is 3 MiB. If UniXNU does not fit, the EDK II build stops. Then either drop vendor drivers that UniXNU does not need (USB, charger) the way sheng's `SHENG_VENDOR_HW = 0` does, or grow the FD into the 576 KiB gap below the UEFI stack (`pipa.toml` and the "UEFI FD" descriptor together).
- **Vendor drivers and the panel.** On sheng the vendor clock stack blanked the panel; upstream reports a working display on pipa, but not together with UniXNU.
- **ABL acceptance** of a 192 MiB ramdisk in a header v1 image is unverified on pipa; the stock boot image is header v3.
- **userdata.** The root image is written with `fastboot flash userdata`, which erases Android's data partition (done on 2026-10-23; the full backup restores Android).

## Next steps

1. GPU: the GMU's SMMU context (NativeAdreno's GMU memory layout fits the A650's), then the GMU cold start with `a650_gmu.bin` (A650 differences from the A740: DTCM at 0x10004000, the a650 RSC sequence, the PDC sequence held by the AOP), the HFI startup and the request that powers GX; then the a6xx hardware and CP initialisation of UniXNU-tb128fu's NativeAdreno fork with `a650_sqe.fw` and the zap shader.
2. Display: the CPU copy to the firmware frame buffer has no vertical sync (tearing, as on UniXNU-tb128fu); later the DSI/MDSS path or a GPU copy.
3. A UFS log beside the ramoops zone, for resets that do not keep memory.

## License

BSD-2-Clause-Patent. See [LICENSE](LICENSE). `make-pipa-adt.py` is derived from `make-sheng-adt.py` of UniXNU-sheng; `build-arm-driver.py` and the two panic-screen files are copies from it. `ShengPlatformPower.cpp` comes from it under MIT (its SPDX header).
