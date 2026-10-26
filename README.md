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
| `drivers/` | This board's drivers under UniXNU's component names (UniXNU `components.toml`): Adreno650, PipaUfs and PipaNovatekTouch, the forks of Adreno740, ShengUfs and ShengNovatekTouch for this board's hardware; AdrenoMetal, GopFramebuffer and ArmCpu (branch `pipa` of the forks under the same names); ShengHidInput as UniXNU publishes it |
| `scripts/build-adreno650.py`, `scripts/build-gop-legacy-display.py` | The Adreno650 archive, and the display archive with this board's key, touch and GPU probes |
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

State on 2026-10-24: iPadOS 17.7.11 runs to the home screen on all eight cores at their top clocks (two scheduler clusters, E for the A55s and P for the A77s), from the UFS root, with the display on the firmware frame buffer. Touch (swipe to unlock) and the power and volume keys (lock, wake) work. The Adreno 650 runs under its GMU and the UI renders through Metal (NativeAdreno publishes the GPU, the Metal plugin translates to Vulkan for Turnip, the GPU copies each frame to the frame buffer) at 40 to 54 frames a second while the home screen pages move (the user saw close to 60 on most pages and about 35 when swiping to the App Library), with rounded corners drawn wrong in GMEM passes. Without the plugin's publish file the UI renders on the CPU as before. No audio or network yet.

One round is one command, with no keys: `scripts/kernel-round.sh [--no-build | --build-only] [--twice] [--arg NAME=VALUE]... [TAIL_LINES]` builds the kernel, payload and boot image against the local IPSW, `fastboot boot`s it, waits until the kernel restarts the tablet into fastboot by itself (a warm reset that keeps the ramoops log: 10 s after a panic, after the third SpringBoard/backboardd crash, or 240 s after boot), reads the log back and prints the kernel's own lines (display, cores, CPU levels, crashes) and the end. `--twice` after a change of `pipa_defconfig`; `--arg sheng-data-reset=1` once after a change that MobileGestalt caches in the Data overlay (the UI scale).

What makes it up (see `configs/pipa_defconfig` and `board.toml` for every switch and argument):

- `package/BootRamoops.c`: the kernel log in the stock ramoops console zone (stages, process events, exits and crash lines, the message buffer every 100 ms, display and core counters every 10 s), the restart into fastboot (uefi-pipa's RamoopsLogLib sequence), the panic hook the panic screen calls, and `pipa-cpufreq=1` (the EPSS domains to their top levels).
- `drivers/PipaUfs` (branch `pipa`): the SM8250 host, keeping UEFI's SMMU pass-through for streams 0xe0/0x4e0; `native-ufs-probe=16` surveys it without the root.
- userdata: UniXNU-tb128fu's root image (`out/tb128fu/userdata/userdata-root.sparse.img`, the same IPSW and owner key), flashed with `fastboot flash userdata`; the owner root chain, Data seed and trust caches as on UniXNU-tb128fu.
- The display stack of UniXNU-tb128fu without its GPU and touch probes (`drivers/GopFramebuffer` branch `pipa`, `package/LegacyDisplayProbe.cpp` and the key store and HID probes), at UI scale 2 (900x1440 points).
- `drivers/ArmCpu` (branch `pipa`): all eight cores, IPIs as SPIs at the distributor's top eight INTIDs (952..959).
- `package/BootFaultProbe.c` and `BootWatchdogPolicy.c` of UniXNU-tb128fu: crash records and the launchd job policy.
- Touch (runs on the tablet since 2026-10-24: axes and range right, about 120 reports a second while a finger moves): `package/TouchProbe.cpp` (`pipa-touch=<stage>`, 5 in `board.toml`) drives the Novatek NT36532 through UniXNU's `drivers/QcomGeniSpi` and `drivers/PipaNovatekTouch` (branch `pipa`: the point reports of this firmware). Stages: 1 a read-only snapshot of clocks and pins, 2 the QUP wrapper 0 clocks and the SPI pins, 3 the GENI SPI engine, a loopback test and the chip id, 4 the firmware download and its info block, 5 the report loop on the attention line (TLMM 39, latched without an interrupt), publishing to the HID input. The board facts and their sources are in `package/TouchBoard.h` (host test `package/TouchBoardTests.c`). The firmware is `novatek_nt36532_m82_fw_tm.bin` from the stock kernel's built-in firmware table (the panel's maker 0x36 pairs with the `_tm` image), kept in `.cache/firmware/touch` outside Git. The kernel log shows the touch rows every 10 s and each finished stroke with its start and end points, for checking the axes; The firmware reports 16-bit coordinates in tenths of a pixel (its info block: 18000 x 28800, Android's range 0..17999 x 0..28799); the driver scales them by that range.
- Keys (run on the tablet since 2026-10-24): `package/Buttons.c` (`pipa-buttons=1`) reads the PM8150 PON's status (power, volume down) and PM8150 GPIO6 (volume up, active low) over SPMI every 10 ms, read only, and passes the pressed set to the HID input (power, volume up and down of its Consumer report). Its first log line shows how the firmware left GPIO6 (an input with a pull-up is mode 0, pull 0, enable 0x80); every change is logged. Decoding and debounce: `package/ButtonsCore.h`, host test `package/ButtonsTests.c`.
- Battery (not yet run on the tablet): `package/NativeBattery.cpp` publishes an `IOPMPowerSource`, the class powerd in the userland matches to build `IOPSGetPowerSourcesInfo`, from `LegacyDisplayProbe.cpp`'s late probe so the object exists before launchd starts powerd. The PM8150B's gen4 gauge and SMB5 charger (SPMI USID 2) are polled every 10 s through the same arbiter observer channels as the keys: monotonic SOC (0x4009 with its copy at 0x400A), battery voltage and current (0x41A0/0x41A2 with the copies at 0x41A6/0x41A8), charger stage and enable (0x1006/0x1042), VBUS presence (0x1310) and the battery-missing status (0x1210). Temperature is the gauge's SRAM word 328, read by claiming the interleaved memory access (the driver's only write path, through the EE 0 channel, released on every exit), with the batt-info registers 0x4150/0x4151 as the fallback; the row names the source it used. `pipa-battery=0` disables the driver and `pipa-battery=2` keeps it to reads; it is on by default. Facts, register addresses and the unit conversions are in `package/NativeBatteryBoard.h` (host test `package/NativeBatteryTests.c`), from the device tree's PM8150B nodes and Linux's qpnp-fg-gen4.c, fg-reg.h, fg-memif.c and smb5-lib.c, smb5-reg.h. The kernel log shows the row after every change: `battery: 99%(r254) 4485mV -75mA 36.5C stg4 en1 ext1 bat1 wd53=33 t:sram chg` (the value in parentheses is the gauge's raw monotonic SOC, 0..255; stage 4 is taper; `t:sram` or `t:reg` names the temperature source and `t:err(<step>,<code>)` means neither answered (the step is where the gauge's SRAM read stopped)). A one-time `NativeBattery: published ...` and `NativeBattery: IOPMPowerSource match 1` (the kernel's own `IOServiceMatching("IOPMPowerSource")` found the object, which is how powerd looks for it) precede it. Nothing pets the charger's bark/bite watchdog, and the row's `wd53` value is what the firmware left in its configuration, so if the firmware enabled it the charger may pause after its bite timeout.
- GPU power and clocks (run on the tablet since 2026-10-24): `package/GpuProbe.cpp` (`pipa-gpu=<mode>`, 6 in `board.toml`) after UniXNU-sheng's GPU power probe, with the SM8250 facts in `package/GpuPowerBoard.h` (host test `package/GpuPowerBoardTests.c`). Mode 6 powers the CX domain (it was collapsed), turns on the GPU SMMU's clocks and reads its global space (translation on, no stream mapped: the firmware leaves the GPU alone), reads the AOP command DB at 0x80860000 (gfx, mx and cx level lists; SH0 0x50004, MC0 0x50000, ACV 0x5007c, CN0 0x50074) and brings up the GMU clocks at 200 MHz from GPLL0's divided output; the GMU stays in reset with GX off, ready for a cold start. The rows go to the kernel log as `gpu:` lines.
- GMU cold start (runs on the tablet since 2026-10-24, `pipa-gpu=7` in `board.toml`: 16 firmware blocks loaded and read back, firmware version 0x20010008 ready with 0x100, the HFI queues ready, the four startup messages acknowledged without error; the GMU then runs with GX off): mode 7 of `package/GpuProbe.cpp` builds the GMU's SMMU context through `drivers/Adreno650` (branch `pipa`, off UniXNU-tb128fu's: the GMU stream 5 under its DT mask 0x400 as two exact routes, 5 and 0x405, to bank 1; `kits/ArmSmmu4K` branch `pipa` accepts free slots whose FAULT S2CR reads 0x000203ff, as this SMMU's do) and runs `package/A650Gmu.c` (host test `package/A650GmuTests.c`, which also loads the real firmware): Linux v6.6 a6xx_gmu.c's a650 sequence (RSC setup with the PDC sequence held by the AOP, firmware blocks into ITCM, DTCM at 0x10004000 and the icache region, the boot registers, release, HFI start) and the HFI startup messages of UniXNU's `kits/AdrenoFirmware`, with performance votes from the command DB's level lists and a650_build_bw_table's single "off" bandwidth level. The GPU, GMU, SQE and zap images come from the tablet's vendor partition (`/firmware/a650_*`, extracted with `dump.erofs --offset=0x168100000 --cat --path=/firmware/<file> super.img`), kept in `.cache/firmware/gpu` outside Git; `scripts/build-adreno650.py` builds `libNativeAdreno.a` (`CONFIG_XNU_NATIVE_ADRENO`).
- GX power and the first GPU command (run on the tablet since 2026-10-24, `pipa-gpu=9` in `board.toml`): mode 8 sends the HFI GX_BW_PERF_VOTE (305 MHz, `pipa-gpu-perf=<1..7>`) and the OOB GPU_SET request (`A650GmuPowerGx`); GX came on with the request acknowledged on the second poll. Mode 9 then builds the GPU's SMMU context (bank 0, streams 0, 1, 0x400 and 0x401 of its iommus `<0 0x401>`), runs Linux v6.6 hw_init for the A650 (no clock gating, no APRIV; UBWC with highest bank bit 16; a650_protect), loads `a650_sqe.fw`, starts ring 0 (NO_UPDATE) and CP_ME_INIT, and has the GPU write a marker that the CPU reads back: 0xA6500C0D, the ring idle and no fault.
- Zap shader and GPU interrupt (run on the tablet since 2026-10-24, `pipa-gpu=10` in `board.toml`): mode 10 loads `a650_zap.elf` through PAS 13 into `pil_gpu_region@8681a000` (init_image, mem_setup, segment copy, auth_and_reset, all answered 0; the metadata lies above 4 GiB, since almost none of XNU's memory is below it, and TrustZone takes it) and sends CP_SET_SECURE_MODE(0); then it attaches the GPU interrupt (SPI 300 through ArmCpu) and a CP_NOP fence raises it.
- Metal (run on the tablet since 2026-10-24, `pipa-gpu=12`, `sheng-gpu-addon=1`, `sheng-fb-gpu=1`): mode 11 publishes the GPU for user clients (NativeAdrenoSubmit on ring 0, the CP always-on counter as its clock, the A650's facts: chip id 0x06050002, GMEM 1.125 MiB at base 0 as Linux reports it, highest bank bit 16, eight-channel macrotiles as RBBM_NC_MODE_CNTL reads at reset) and runs the user-client path twice from the kernel (4/4 both times); mode 12 registers the staged accelerator for the original Metal discovery. hw_init now also sets Linux's a650_hwcg clock gating, and NativeAdreno's kernel buffers lie at 32 MiB and up (its pipa branch), clear of the GMEM window at 0. The Metal plugin, Turnip and adrenoprobed come from a GPU addon: by default UniXNU-tb128fu's build of the same NativeMetal code (`scripts/kernel-round.sh`, `PIPA_GPU_USERSPACE`); this tree builds its own (`build-adreno-metal-plugin.py` in `out/pipa/tree` after cloning UniXNU-tb128fu's Mesa, venv, Turnip and bridge inputs, then `make-owner-trust-bundle.py` and `make-gpu-addon.py`).
  hw_init sets CP_APRIV_CNTL as Linux does on the A650 (`pipa-gpu-apriv=0` leaves it off). Without it every read through the UCHE by the shaders and the texture pipe ended in UCHE_OOB_ACCESS, with no GPU SMMU fault: Turnip's check filled a buffer (2D engine, no read) but its compute shader wrote none of its 1024 words and its image-to-buffer copy nothing (`T s0 r0 v512 c0 g0/4096 wdeadbeef u1024 p5a5a5a5a`), and Metal's frames were all zero (a kernel sample of the frame buffer: 0 of 1536 points lit), a black screen. The UCHE registers matched Linux v6.6 throughout; clock gating, the GMEM window at 16 MiB or at 0, the kernel buffers' place and linux-firmware's SQE made no difference. With APRIV the same check passes (`T s0 r0 v1024 c1024 g512/0 w1000 u0 pffff0000`), the Metal bridge's self-test and frontend checks pass in SpringBoard (`M s0 c0 r0 i0 q167 k4/11 p15/8 F0`), and with the publish file backboardd renders the lock screen through Metal: the GPU's first copy to the scanout reads back equal, and the kernel's frame buffer sample is the same as the CPU-rendered lock screen's (13 of 1536 points lit, the same sum). Turnip picks GMEM or system memory per pass (`pipa-turnip-debug=nocb`; before APRIV its GMEM passes hung on a CP_DRAW_INDX_OFFSET).
  Frame rate: `--arg pipa-touch=10245` scripts 40 strokes for a round nobody watches (package/TouchProbe.cpp, after UniXNU-tb128fu's: 30 s after the first GPU hand-off frame a swipe up from the bottom edge unlocks, then strokes right to left and back turn the home screen's pages). Such rounds (2026-10-24) unlock and turn pages with every frame copied to the scanout by the GPU (1257 copies and more, no failure). While the UI moves (`display: pacing`: swaps less than 200 ms apart; the 10 s swap counts mix motion with the rest between strokes) three rounds on the E/P clusters ran at 53.9, 40.5 and 41.3 frames a second (the first: 1510 frames in 28.0 s, 56 % of the gaps one 60 Hz refresh, 42 % two, submit to retire 22 ms on average; the other two 40 ms), and one with every pass in system memory at 38 (`sysmem,nocb`: 1721 frames in 45.3 s, 178 gaps over 50 ms; submit to retire 39 ms). Between the fast round and the slow ones neither the GPU's time per command buffer (the scanout copy 4.0 ms in both) nor the render server's CPU time differs; commit to completion does (13.5 ms on average against 26.7), and so does the plugin's scanout copy per frame (13 ms against 22). The kernel's frame buffer pictures (`fbpic:`, 45 x 36 characters of luma) show why: the rounds turned between different pages (the slow ones between the first page, with widgets, and the second, with four folders), so they did not measure the same work.
  Rounded corners: in GMEM passes the widget platters' rounded corners are cut off diagonally (the user's photo), and a 64 x 64 crop of the frame buffer at full resolution (`fbcrop:`) shows the corner made of 4 x 4 and 8 x 4 pixel blocks with holes; with `sysmem,nocb` the corners are right (the user saw it). Withholding the Apple GPU families (`pipa-metal-withhold=apple-families`, the accelerator's NativeMetalWithhold) changed nothing. Candidates: what GMEM passes do that system-memory ones do not (LRZ, UBWC, binning, the stores of unaligned render areas and of S8 through the texture path at GMEM base 0).
  The render server stopped committing after its first frames in 2 of about 20 rounds, without a crash and with nothing outstanding on the GPU (not tied to a cold shader cache: a round with a fresh Data overlay ran through). BootProcessProbe.c and BootUiTraceProbe.c (UniXNU-tb128fu's) now log every process pass (`process:`) and backboardd's and SpringBoard's thread stacks 200 s after the log zone is mapped and before the restart (`stacks:`), for the next time. On one scheduler cluster the render server's thread committed 39 % of its frames from a Cortex-A55 and took 17.8 ms of CPU per frame, about 46 frames a second (the bridge's `U` row: 21.8 ms between frames); on the E/P clusters every frame comes from an A77 in 10 to 11 ms of CPU. The GPU is busy 18 to 46 % of each 2 s beat (GMU busy counter, `busy` in the LIVE row, per mille); the plugin's telemetry (`metal table:` rows) shows the heaviest page-turn frame, 8 render passes with 141 draws over 16.9 million pixels, in 15 ms of GPU time, and the scanout copy in 4 ms, but commit to completion takes 27 ms on average for 7.5 ms of GPU time (one ring: the render server's buffers and the scanout copies queue behind each other). One round with `sysmem,nocb` stopped after the unlock swipe: the render server committed 7 command buffers and no more, with nothing outstanding on the GPU and no crash; the next such round ran through. The SQE is linux-firmware's `qcom/a650_sqe.fw` (version 0x112, in `.cache/firmware/linux-firmware`): the vendor partition's is version 0x093, which Linux refuses (at least 0x095 for its security fixes). The kernel log carries the stack's rows (`gpu stack:`), the GPU SMMU's faults (`FAULTS`), the UCHE registers (`UCHE`), the GPU's register blocks after the secure exit and at the first UCHE_OOB_ACCESS (`gpu regs`), and a frame buffer sample (`framebuffer:`).

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

1. GPU: the rounded corners of GMEM passes (above); the stutter (the gaps alternate between one and two 60 Hz refreshes; commit to completion 27 ms for 7.5 ms of GPU time: the queueing on one ring behind the scanout copy and the completion path; the App Library at about 35 frames a second); the render server's rare stop after its first frames; empty widget platters; the scanout copy's one failure in a check round (`PF 1/1 r3`); bandwidth votes.
2. Memory: XNU owns one contiguous range, 5.75 GiB of the second bank since 2026-10-24 (from 0xffc20000; 4 GiB before). 7.75 GiB went to the third bank (0x800000000) and the kernel stopped after the blue entry bar, without a log: a physical base at 32 GiB is not handled yet. All 16 GB needs several physical ranges in XNU's pmap and VM start-up.
3. Display: the CPU copy to the firmware frame buffer has no vertical sync (tearing, as on UniXNU-tb128fu); later the DSI/MDSS path or a GPU copy.
4. A UFS log beside the ramoops zone, for resets that do not keep memory.

## License

BSD-2-Clause-Patent. See [LICENSE](LICENSE). `make-pipa-adt.py` is derived from `make-sheng-adt.py` of UniXNU-sheng; the two panic-screen files are copies from it. `ShengPlatformPower.cpp` comes from it under MIT (its SPDX header).
