/*
 * SPDX-FileCopyrightText: nvidia-driver-injector contributors
 * SPDX-License-Identifier: MIT
 *
 * nv-tb-egpu-pcie.h — Thunderbolt-eGPU shared PCIe/AER/WPR2 register-read
 * primitives (addon layer A1).
 *
 * These are pure register / config-space reads with no recovery-state
 * coupling.  They are consumed by:
 *   - A3 (nv-tb-egpu-recover.c): WPR2 read + AER trigger capture
 *   - A2 (nv-tb-egpu-qwd.c): AER trigger capture at watchdog detection
 *   - A3 (nv-pci.c): AER trigger capture at err_handler callbacks
 *
 * No caller exists in A1 itself — the functions are declared here
 * (non-static) so the compiler does not emit unused-function warnings
 * when the translation unit is compiled before the callers land.
 */

#ifndef _NV_TB_EGPU_PCIE_H_
#define _NV_TB_EGPU_PCIE_H_

#include <linux/pci.h>
#include <linux/types.h>

/*
 * Compact AER snapshot written by tb_egpu_dump_aer_trigger_event and
 * stored by the A2 Q-watchdog in its per-device state (nv-tb-egpu-qwd.h
 * embeds this struct in struct tb_egpu_qwd).  Defined here — in the
 * foundation layer — because A1 is the only writer of the struct fields;
 * A2 only embeds and reads it.
 *
 * Note on torn reads: the sysfs reader in A2 reads the struct without a
 * lock.  Acceptable for diagnostic-only telemetry — userspace consumers
 * should re-read on unexpected values.
 */
struct tb_egpu_qwd_aer_snapshot
{
    u32 gpu_aer_uesta;
    u32 gpu_aer_cesta;
    u32 br_aer_uesta;
    u32 br_aer_cesta;
    u32 root_aer_uesta;
    u32 root_aer_cesta;
    u32 root_rootsta;
    u16 dpc_status;
    u8  valid;  /* 0 = never populated; 1 = populated at least once */
};

/*
 * WPR2 status register: NV_HUBMMU0_PRI_BASE + NV_HUBMMU_PRI_MMU_WPR2_ADDR_HI
 * for Blackwell GB100/GB202 (per published headers
 * src/common/inc/swref/published/blackwell/gb100/{hwproject.h,
 * dev_hubmmu_base.h}).  DRF field _VAL is bits [31:4]; non-zero means
 * WPR2 is up.
 */
#define TB_EGPU_PCIE_WPR2_REG_OFFSET   0x88a828u
#define TB_EGPU_PCIE_WPR2_VAL_MASK     0xfffffff0u    /* bits 31:4 */

/*
 * Read the raw WPR2 status register at the GB100/GB202 published offset.
 * Returns 0 on success and stores the raw value in *raw_out; returns
 * -errno on ioremap failure.  The mapping is page-bounded and released
 * before return — no persistent state.
 */
int tb_egpu_pcie_read_wpr2(u64 bar0_phys, u32 *raw_out);

/*
 * Walk up the PCIe topology from start toward the host root port.
 * Iterates pci_upstream_bridge until pci_pcie_type == ROOT_PORT
 * (bounded to 8 hops).  Returns the root-port pci_dev, or NULL if
 * not found within the hop limit.
 */
struct pci_dev *tb_egpu_pcie_walk_to_root_port(struct pci_dev *start);

/*
 * Read the DPC (Downstream Port Containment) extended capability from
 * pdev.  Writes *present_out = false and zeros the status/ctl outputs
 * if the capability is absent.
 */
void tb_egpu_pcie_read_dpc_state(struct pci_dev *pdev,
                                    bool *present_out,
                                    u16 *dpc_status_out,
                                    u16 *dpc_ctl_out);

/*
 * Read the full AER extended capability from pdev.  All output pointers
 * are zeroed before the read; NULL pointers for optional fields are safe
 * (they are skipped).
 */
void tb_egpu_pcie_read_aer_full(struct pci_dev *pdev,
                                   int *pos_out,
                                   u32 *uesta, u32 *uemsk, u32 *uesvrt,
                                   u32 *cesta, u32 *cemsk,
                                   u32 hdrlog[4],
                                   u32 *rootcmd, u32 *rootsta,
                                   u32 *errsrc);

/*
 * Trigger-event AER capture (legacy "Mode B telemetry S1").
 *
 * Walks GPU -> upstream bridge -> true host root port (bounded to 8
 * hops via tb_egpu_pcie_walk_to_root_port), dumps full AER + DPC +
 * link state in one printk block, and optionally writes a compact
 * snapshot to *out for sysfs persistence (the qwd S3 surface in addon
 * A2 owns *out).
 *
 * Called from:
 *   - Q-watchdog detection (nv-tb-egpu-qwd.c, inside the detected_logged
 *     latch), with out=&qwd->last_aer
 *   - nv_pci_error_detected and nv_pci_mmio_enabled / nv_pci_cor_error_detected
 *     callbacks, with out=NULL (no per-callback snapshot persistence)
 *
 * Internal-only — declared here for kernel-open consumers; not exported
 * across module boundaries.
 */
void tb_egpu_dump_aer_trigger_event(struct pci_dev *gpu_pdev,
                                    const char *trigger,
                                    struct tb_egpu_qwd_aer_snapshot *out);

#endif /* _NV_TB_EGPU_PCIE_H_ */
