/*
 * SPDX-FileCopyrightText: nvidia-driver-injector contributors
 * SPDX-License-Identifier: MIT
 *
 * nv-tb-egpu-close.c — Thunderbolt-eGPU close-path nominal telemetry (addon A4).
 *
 * RM-side close-path markers.  Four nv.c call sites (close-entry,
 * pre-stop, post-shutdown, close-exit) each emit a single line.  On the
 * last-close transition (usage_count → 0 / is_last_close=true), a
 * minimal hardware-health snapshot is added:
 *
 *   PMC_BOOT_0   ioremap + ioread32 on BAR0 — confirms the GPU responds
 *                to MMIO at the point of last-close.
 *   WPR2 status  via A1's tb_egpu_pcie_read_wpr2 — the single field
 *                that distinguishes a clean close (WPR2=0) from the
 *                stuck-WPR2 failure mode we recover from in A3.
 *   Health       one-word verdict: "wpr2_up:YES" or "wpr2_up:no".
 *
 * Observability scope (per design 2026-05-22-addon-recarve-design.md):
 * nominal bar only.  LnkSta, AER Unc/Cor, and tb_egpu_dump_aer_trigger_event
 * are intentionally absent — those are investigation-grade and belong in
 * the dissolved P6 DIAG surface, not in production soak logs.
 *
 * MISSION-1 v4 relationship to C5's canonical sink-side log
 * (cascade-class-design-v4.md): C5's cleanupGpuLostStateAtomic primitive
 * is the single source of truth for "GPU N was just classified lost via
 * detector class X" — it emits one canonical NV_PRINTF per
 * (gpu, detector_class) per kernel-module lifetime.  A4's lines here are
 * the ORTHOGONAL close-path nominal observability ("did the close path
 * execute, and what was the hardware state at last-close?") and MUST NOT
 * duplicate the "GPU lost" marker.  v1's format strings already satisfy
 * this by construction (none of A4's lines emit "GPU lost" wording);
 * future additions to this file MUST preserve the non-duplication
 * invariant — add only the site-attributed state-delta payload, never the
 * basic "GPU N lost" line.
 *
 * Cross-module surface:
 *   tb_egpu_get_gpu_pdev — exported for nvidia-uvm.ko (A4 UVM side)
 *   tb_egpu_close_diag_pdev — exported for nvidia-uvm.ko (A4 UVM side)
 *
 * Passive instrumentation only — ioremap+ioread32 + PCI config-space
 * reads.  No DMA, no register writes.
 */

#include "nv-tb-egpu-close.h"
#include "nv-tb-egpu-pcie.h"   /* A1: tb_egpu_pcie_read_wpr2, WPR2 macros */
#include "nv-linux.h"
#include "nv-pci-types.h"

#include <linux/io.h>
#include <linux/pci.h>
#include <linux/printk.h>

/* =======================================================================
 * Cross-module pdev lookup (EXPORT_SYMBOL_GPL — nvidia-uvm.ko consumer).
 * =======================================================================
 */

struct pci_dev *tb_egpu_get_gpu_pdev(void)
{
    struct pci_dev   *out = NULL;
    nv_linux_state_t *nvl;

    LOCK_NV_LINUX_DEVICES();
    for (nvl = nv_linux_devices; nvl != NULL; nvl = nvl->next)
    {
        if (nvl->pci_dev)
        {
            out = pci_dev_get(nvl->pci_dev);
            break;
        }
    }
    UNLOCK_NV_LINUX_DEVICES();
    return out;
}
EXPORT_SYMBOL_GPL(tb_egpu_get_gpu_pdev);

/* =======================================================================
 * Pdev-based passive hardware-health snapshot.
 *
 * Re-scoped to nominal: PMC_BOOT_0 + WPR2 + one-word health verdict.
 * LnkSta and AER register dumps from legacy 0005 are absent by design
 * (investigation-grade; resolved to P6 which is dissolved).
 * =======================================================================
 */

void tb_egpu_close_diag_pdev(struct pci_dev *pdev, const char *site)
{
    u64           bar0_phys;
    u32           pmc_boot_0 = 0xdeadbeefu;
    u32           wpr2_raw   = 0xdeadbeefu;
    int           wpr2_rc;
    void __iomem *map_pmc;
    bool          pmc_ok = false;

    if (!pdev)
    {
        nv_printf(NV_DBG_ERRORS,
            "tb_egpu [CLOSE]: site=%s pdev=NULL — cannot read\n", site);
        return;
    }

    bar0_phys = pci_resource_start(pdev, 0);
    if (bar0_phys == 0)
    {
        nv_printf(NV_DBG_ERRORS,
            "tb_egpu [CLOSE]: site=%s bar0=0 — skipping\n", site);
        return;
    }

    /* PMC_BOOT_0 (BAR0 offset 0). */
    map_pmc = ioremap(bar0_phys, PAGE_SIZE);
    if (map_pmc)
    {
        pmc_boot_0 = ioread32(map_pmc);
        iounmap(map_pmc);
        pmc_ok = true;
    }

    /* WPR2 via the A1 shared helper. */
    wpr2_rc = tb_egpu_pcie_read_wpr2(bar0_phys, &wpr2_raw);

    nv_printf(NV_DBG_ERRORS,
        "tb_egpu [CLOSE]: site=%-15s pdev=%s bar0=0x%llx "
        "PMC_BOOT_0=%s%08x WPR2=%s%08x wpr2_up:%s\n",
        site, pci_name(pdev),
        (unsigned long long)bar0_phys,
        pmc_ok ? "0x" : "MAPFAIL:", pmc_boot_0,
        (wpr2_rc == 0) ? "0x" : "MAPFAIL:", wpr2_raw,
        ((wpr2_raw & TB_EGPU_PCIE_WPR2_VAL_MASK) != 0) ? "YES" : "no");
}
EXPORT_SYMBOL_GPL(tb_egpu_close_diag_pdev);

/* =======================================================================
 * RM-side close-path nominal marker.
 * =======================================================================
 */

void tb_egpu_close_diag(nv_linux_state_t *nvl,
                        const char *site,
                        long usage_count,
                        bool is_last_close)
{
    nv_state_t *nv;

    if (!nvl)
        return;

    nv = NV_STATE_PTR(nvl);

    NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
        "tb_egpu [CLOSE]: site=%-15s usage_count=%ld%s\n",
        site, usage_count, is_last_close ? " (LAST-CLOSE)" : "");

    if (is_last_close && nvl->pci_dev)
    {
        /*
         * Minimal hardware-health snapshot at last-close.  PMC_BOOT_0 +
         * WPR2 + verdict — enough to confirm whether the GPU was alive
         * and WPR2 was clean at the point of close, without the full
         * LnkSta + AER investigation-grade dump.
         */
        tb_egpu_close_diag_pdev(nvl->pci_dev, site);
    }
}
