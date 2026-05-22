/*
 * SPDX-FileCopyrightText: nvidia-driver-injector contributors
 * SPDX-License-Identifier: MIT
 *
 * nv-tb-egpu-qwd.h — Thunderbolt-eGPU Q-watchdog (addon layer A2).
 *
 * Per-device kthread that probes NV_PMC_BOOT_0 via direct volatile MMIO
 * on a fixed interval. On a dead-bus return (0xFFFFFFFF), declares the
 * GPU disconnected through os_pci_set_disconnected — the same kernel
 * propagation Q-active uses from the ioctl path, but driven by an
 * active heartbeat that fires regardless of which subsystem caused the
 * wedge. Closes the DMA-path Mode B detection gap that Q-active alone
 * misses; see docs/lever-catalog.md (Lever Q) for the full taxonomy.
 *
 * Public surface (per device, via PCI sysfs):
 *
 *   tb_egpu_qwd_cycles                  cumulative kthread iterations
 *   tb_egpu_qwd_detections              dead-bus episodes detected
 *   tb_egpu_qwd_last_detection_jiffies  jiffies at first detect of last episode
 *   tb_egpu_qwd_last_pmc_boot_0         PMC_BOOT_0 value at last detect
 *   tb_egpu_qwd_last_aer_summary        compact AER + DPC snapshot at last detect
 *
 * Module parameters (runtime-toggleable, /sys/module/nvidia/parameters):
 *
 *   NVreg_TbEgpuQwdEnable      1 = kthread runs, 0 = sleeps idle
 *   NVreg_TbEgpuQwdIntervalMs  probe period (clamped to [10, 60000])
 *
 * See nv-tb-egpu-qwd.c for design rationale, lifecycle notes, and the
 * Heisenbug-acknowledgement reasoning for an active observer in the
 * presence of an observability-sensitive bug.
 *
 * A1/A2 dependency: struct tb_egpu_qwd_aer_snapshot is defined in
 * nv-tb-egpu-pcie.h (addon A1) and included below. A2 embeds it in
 * struct tb_egpu_qwd; A1's tb_egpu_dump_aer_trigger_event() populates it.
 */

#ifndef _NV_TB_EGPU_QWD_H_
#define _NV_TB_EGPU_QWD_H_

#include <linux/atomic.h>
#include <linux/types.h>

/*
 * Pull in struct tb_egpu_qwd_aer_snapshot and the tb_egpu_dump_aer_trigger_event
 * declaration from A1. A2 must not re-define the struct.
 */
#include "nv-tb-egpu-pcie.h"

struct task_struct;
struct nv_linux_state_s;
typedef struct nv_linux_state_s nv_linux_state_t;

struct tb_egpu_qwd
{
    struct task_struct *thread;
    atomic_t            cycles;
    atomic_t            detections;

    /* Persistent S3 detection state — populated on detect events. */
    unsigned long                       last_detection_jiffies;
    u32                                 last_pmc_boot_0;
    struct tb_egpu_qwd_aer_snapshot     last_aer;
};

int  tb_egpu_qwd_init(nv_linux_state_t *nvl);
void tb_egpu_qwd_stop(nv_linux_state_t *nvl);

#endif /* _NV_TB_EGPU_QWD_H_ */
