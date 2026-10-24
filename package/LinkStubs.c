// SPDX-License-Identifier: BSD-2-Clause-Patent
// No-op definitions of the probe hooks the patched XNU references while pipa_defconfig leaves their component out.
// Each hook runs only on its boot argument, which board.toml does not pass; the kernel link needs the symbol anyway.

// offline-component-load-probes.patch, bsd_init: called on offline-legacy-register=1 (NativeLegacyDisplay). Weak: with
// CONFIG_XNU_NATIVE_LEGACY_DISPLAY the display archive's LegacyDisplayProbe.cpp defines the real probe.
void native_legacy_display_probe(void);
__attribute__((weak)) void native_legacy_display_probe(void)
{
}

// NativeHidInput registers its button callback with a board button driver (power, volume up/down). Weak: Buttons.c (in the
// display archive) defines the real one. The kernel is linked statically, where an undefined weak reference does not link.
void native_buttons_set_sink(void (*sink)(void *, unsigned), void *context);
__attribute__((weak)) void native_buttons_set_sink(void (*sink)(void *, unsigned), void *context)
{
  (void)sink;
  (void)context;
}

// NativeAdrenoPublish.cpp (CONFIG_XNU_NATIVE_ADRENO) hands the Metal plugin's exported pipeline caches to the board log, which
// UniXNU-tb128fu's BootLog.c writes into NativeUfs's export region. pipa has no UFS write path yet: -4 is BootLog.c's answer
// when there is no export region.
typedef unsigned int u32;
int boot_log_export_put(u32 generation, u32 total, u32 offset, const void *data, u32 size);
__attribute__((weak)) int boot_log_export_put(u32 generation, u32 total, u32 offset, const void *data, u32 size)
{
  (void)generation;
  (void)total;
  (void)offset;
  (void)data;
  (void)size;
  return -4;
}
