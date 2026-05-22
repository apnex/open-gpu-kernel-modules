/*
 * SPDX-FileCopyrightText: nvidia-driver-injector contributors
 * SPDX-License-Identifier: MIT
 *
 * nv-tb-egpu-close.h — Thunderbolt-eGPU close-path nominal telemetry (addon A4).
 *
 * Event-triggered telemetry at RM close-path transitions.  A single log
 * line is emitted at each of the four nv.c call sites; on the last-close
 * transition (usage_count → 0) a minimal hardware-health snapshot is
 * also captured via tb_egpu_close_diag_pdev:
 *
 *   - PMC_BOOT_0 (ioremap + ioread32 on BAR0)
 *   - WPR2 status (via A1's tb_egpu_pcie_read_wpr2)
 *   - one-word health verdict (wpr2_up: YES / no)
 *
 * This is intentionally the *nominal* bar: enough to answer "was the GPU
 * alive at last-close?" in a production soak log, without the full
 * LnkSta + AER multi-register investigation-grade dump that lives in P6
 * (dissolved).
 *
 * Four RM call sites (nv.c):
 *
 *   close-entry     top of nvidia_close_callback after nvl check
 *   pre-stop        inside nvidia_close_callback, under ldata_lock,
 *                   just before nv_close_device
 *   post-shutdown   inside nv_stop_device after nv_shutdown_adapter
 *   close-exit      end of nvidia_close_callback, after nv_close_device
 *
 * Cross-module surface (used by nvidia-uvm.ko, cluster A4's UVM side):
 *
 *   tb_egpu_close_diag_pdev — pdev-based passive state capture,
 *     exported via EXPORT_SYMBOL_GPL.
 *   tb_egpu_get_gpu_pdev — walks nv_linux_devices under the global lock
 *     and returns the first NVIDIA pdev with refcount incremented
 *     (caller MUST pci_dev_put).  Exported via EXPORT_SYMBOL_GPL.
 *
 * Passive instrumentation only — ioremap+ioread32 + PCI config-space
 * reads.  No DMA, no register writes.  Verified non-perturbing 2026-05-08
 * (per memory project_close_path_mitigated_2026_05_08).
 */

#ifndef _NV_TB_EGPU_CLOSE_H_
#define _NV_TB_EGPU_CLOSE_H_

#include <linux/pci.h>
#include <linux/types.h>

struct nv_linux_state_s;
typedef struct nv_linux_state_s nv_linux_state_t;

/*
 * tb_egpu_close_diag — RM-side nominal close-path marker.
 *
 *   nvl          the device's linux state; no-op if NULL
 *   site         short site label (e.g. "close-entry", "post-shutdown")
 *   usage_count  caller's snapshot of nvl->usage_count at the site
 *   is_last_close  caller-determined; true triggers the pdev-based
 *                  hardware-health snapshot via tb_egpu_close_diag_pdev.
 *                  False emits only the one-line marker.
 */
void tb_egpu_close_diag(nv_linux_state_t *nvl,
                        const char *site,
                        long usage_count,
                        bool is_last_close);

/*
 * tb_egpu_close_diag_pdev — pdev-based passive hardware-health snapshot.
 *
 * Reads PMC_BOOT_0 (BAR0 ioremap) + WPR2 (A1 helper) and emits a
 * single log line with a health verdict.  Called from tb_egpu_close_diag
 * on last-close and from the UVM-side close-path helpers cross-module.
 *
 * EXPORT_SYMBOL_GPL — consumed by nvidia-uvm.ko (cluster A4 UVM side).
 */
void tb_egpu_close_diag_pdev(struct pci_dev *pdev, const char *site);

/*
 * tb_egpu_get_gpu_pdev — cross-module pdev lookup for nvidia-uvm.ko.
 *
 * Walks nv_linux_devices under the global lock and returns the first
 * NVIDIA pdev with refcount incremented.  Caller MUST pci_dev_put.
 * Returns NULL if no nvidia.ko-managed device is bound.
 *
 * EXPORT_SYMBOL_GPL — consumed by nvidia-uvm.ko (cluster A4 UVM side).
 */
struct pci_dev *tb_egpu_get_gpu_pdev(void);

#endif /* _NV_TB_EGPU_CLOSE_H_ */
