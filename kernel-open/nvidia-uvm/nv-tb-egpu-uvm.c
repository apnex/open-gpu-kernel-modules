/*
 * SPDX-FileCopyrightText: nvidia-driver-injector contributors
 * SPDX-License-Identifier: MIT
 *
 * nv-tb-egpu-uvm.c — UVM close-path nominal telemetry (addon A4).
 *
 * Companion to the RM-side close-path markers in nv-tb-egpu-close.c.
 * Five UVM lifecycle sites tracked here so we can answer "is the UVM
 * close-path bug class (docs/architecture.md Problem 4) still
 * load-bearing on the current driver stack?" with empirical evidence:
 *
 *   - uvm-open-entry      at end of uvm_open on the success path;
 *                          pre-increment fd_count==0 ⇒ first-after-
 *                          LAST-CLOSE (the canonical post-recovery open)
 *   - uvm-release-entry   at top of uvm_release before any teardown
 *   - uvm-pre-destroy     VA_SPACE branch only, just before
 *                          uvm_va_space_destroy (the destabilising work)
 *   - uvm-post-destroy    VA_SPACE branch only, just after
 *                          uvm_va_space_destroy
 *   - uvm-release-exit    end of uvm_release; post-decrement fd_count==0
 *                          ⇒ canonical LAST-CLOSE
 *
 * On a last-close transition (the open that brings count up from 0 OR
 * the release that brings it back to 0), a PMC_BOOT_0 + WPR2 hardware-
 * health snapshot is captured via tb_egpu_close_diag_pdev.  All other
 * sites emit only the one-line marker — cheap when persistenced or
 * another consumer still holds an fd.
 *
 * Observability scope (per design 2026-05-22-addon-recarve-design.md):
 * nominal bar only.  tb_egpu_dump_aer_trigger_event (full AER multi-hop
 * walk) is intentionally absent — investigation-grade and resolved to
 * the dissolved P6 DIAG surface.
 *
 * MISSION-1 v4 relationship to C5's canonical sink-side log
 * (cascade-class-design-v4.md): C5's cleanupGpuLostStateAtomic primitive
 * is the single source of truth for "GPU N was just classified lost via
 * detector class X".  A4's lines here are the ORTHOGONAL close-path
 * nominal observability ("did the close path execute, and what was the
 * hardware state at last-close?") and MUST NOT duplicate the "GPU lost"
 * marker.  v1's format strings already satisfy this by construction;
 * future additions to this file MUST preserve the non-duplication
 * invariant — add only the site-attributed state-delta payload, never
 * the basic "GPU N lost" line.
 *
 * pdev lookup: tb_egpu_get_gpu_pdev walks nv_linux_devices inside
 * nvidia.ko and returns the first NVIDIA pdev with refcount incremented.
 * Replaces the legacy hardcoded
 * pci_get_domain_bus_and_slot(0, 0x04, PCI_DEVFN(0,0)) — generalises
 * to any PCI topology, single-GPU assumption preserved (the project's
 * scope per memory project_aorus_egpu_setup).
 *
 * Passive-only instrumentation (ioremap+ioread32 inside
 * tb_egpu_close_diag_pdev, PCI config-space reads inside the same).
 * Verified non-perturbing 2026-05-08
 * (per memory project_close_path_mitigated_2026_05_08).
 */

#include "nv-tb-egpu-uvm.h"

#include <linux/atomic.h>
#include <linux/pci.h>
#include <linux/printk.h>

/*
 * Forward declarations of nvidia.ko-exported symbols.  We can't include
 * the nvidia/ tree's nv-tb-egpu-close.h directly because the UVM Kbuild
 * only adds -I$(src)/nvidia-uvm to the include path.
 */
extern struct pci_dev *tb_egpu_get_gpu_pdev(void);
extern void tb_egpu_close_diag_pdev(struct pci_dev *pdev, const char *site);

/*
 * UVM /dev/nvidia-uvm fd refcount.  Incremented in uvm_open success
 * path, decremented in uvm_release exit path.  The kernel guarantees
 * release matches a prior successful open, so mismatched counts are
 * impossible by construction.
 */
static atomic_t tb_egpu_uvm_fd_count = ATOMIC_INIT(0);

/*
 * Shared body — one marker line, and on last-close also a PMC_BOOT_0 +
 * WPR2 snapshot via the cross-module helper.  Centralised so the
 * per-site helpers below stay one-liners.
 */
static void tb_egpu_uvm_emit(const char *site, int fd_count, bool is_last_close)
{
    struct pci_dev *pdev;

    pr_info("tb_egpu UVM [CLOSE]: site=%-18s fd_count=%d%s\n",
            site, fd_count, is_last_close ? " (LAST-CLOSE)" : "");

    if (!is_last_close)
        return;

    pdev = tb_egpu_get_gpu_pdev();
    if (!pdev) {
        pr_info("tb_egpu UVM [CLOSE]: site=%s — no NVIDIA pdev bound; skipping snapshot\n",
                site);
        return;
    }
    tb_egpu_close_diag_pdev(pdev, site);
    pci_dev_put(pdev);
}

void tb_egpu_uvm_close_diag_at_open(void)
{
    int prev = atomic_inc_return(&tb_egpu_uvm_fd_count) - 1;
    /*
     * Increment only on the success path (uvm_open calls us after the
     * NV_OK return is committed).  pre-increment count==0 means this
     * open is the first after a LAST-CLOSE — capture hardware state.
     */
    tb_egpu_uvm_emit("uvm-open-entry", prev + 1, prev == 0);
}

void tb_egpu_uvm_close_diag_at_release_entry(void)
{
    int pre = atomic_read(&tb_egpu_uvm_fd_count);
    /* Pre-decrement read — if count==1, this release will be LAST-CLOSE. */
    tb_egpu_uvm_emit("uvm-release-entry", pre, pre == 1);
}

void tb_egpu_uvm_close_diag_at_pre_destroy(void)
{
    int pre = atomic_read(&tb_egpu_uvm_fd_count);
    tb_egpu_uvm_emit("uvm-pre-destroy", pre, pre == 1);
}

void tb_egpu_uvm_close_diag_at_post_destroy(void)
{
    int pre = atomic_read(&tb_egpu_uvm_fd_count);
    tb_egpu_uvm_emit("uvm-post-destroy", pre, pre == 1);
}

void tb_egpu_uvm_close_diag_at_release_exit(void)
{
    int post = atomic_dec_return(&tb_egpu_uvm_fd_count);
    /* Post-decrement count==0 is the canonical LAST-CLOSE state. */
    tb_egpu_uvm_emit("uvm-release-exit", post, post == 0);
}
