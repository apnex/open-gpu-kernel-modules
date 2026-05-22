/*
 * SPDX-FileCopyrightText: nvidia-driver-injector contributors
 * SPDX-License-Identifier: MIT
 *
 * nv-tb-egpu-recover.h — Thunderbolt-eGPU recovery state machine (addon A3).
 *
 * Per-device in-driver recovery state for two failure paths:
 *
 *   1. Post-rmInit-FAIL with WPR2 stuck up — the cold-cold-boot failure
 *      mode where WPR2 persists from a prior failed boot. Detected by
 *      reading NV_HUBMMU_PRI_MMU_WPR2_ADDR_HI (BAR0 + 0x88a828) after
 *      rm_init_adapter returns failure. Triggers a pci_reset_bus() on
 *      the upstream Thunderbolt bridge from a workqueue context.
 *
 *   2. Runtime AER — the kernel's PCIe error-handler subsystem dispatches
 *      error_detected on PCIe errors. With all gates passing we return
 *      PCI_ERS_RESULT_NEED_RESET; the kernel drives the bus reset and
 *      then dispatches our slot_reset / resume callbacks for verification
 *      and counter reset.
 *
 * Hardening (H1/H2/H3): the post-rmInit-FAIL trigger and the
 * error_detected callback share a gate function that enforces:
 *   - kill-switch (H3): module param NVreg_TbEgpuRecoverEnable + a
 *     persistent file at /var/lib/tb-egpu/recover-killswitch override
 *   - rate limit (H2): NVreg_TbEgpuRecoverMinAttemptIntervalMs since
 *     last fire
 *   - max attempts (H1): NVreg_TbEgpuRecoverMaxAttempts per burst,
 *     resetting only on a verified end-to-end recovery (post-rmInit-OK)
 *     or after NVreg_TbEgpuRecoverSurrenderResetSec of idle
 *
 * The shared PCIe/AER/WPR2 register-read primitives this state machine
 * depends on (tb_egpu_pcie_read_wpr2, tb_egpu_dump_aer_trigger_event,
 * the topology walker, etc.) live in the addon A1 foundation layer —
 * nv-tb-egpu-pcie.{c,h}. This header declares the recovery state machine
 * only; nv-tb-egpu-recover.c #includes nv-tb-egpu-pcie.h for them.
 *
 * Public surface (per device, via PCI sysfs):
 *
 *   tb_egpu_recover_fires               schedule_work invocations
 *   tb_egpu_recover_successes           bus-reset cycles that completed
 *   tb_egpu_recover_surrenders          H1 exhaustions / hard failures
 *   tb_egpu_recover_last_fire_jiffies   last fire timestamp (jiffies)
 *   tb_egpu_recover_force_trigger       (write-only, 0200) test harness
 *
 * Module parameters (runtime-toggleable, /sys/module/nvidia/parameters):
 *
 *   NVreg_TbEgpuRecoverEnable                 master enable (default 0)
 *   NVreg_TbEgpuRecoverMaxAttempts            H1 cap (default 3)
 *   NVreg_TbEgpuRecoverResetSettleMs          post-reset delay (500 ms)
 *   NVreg_TbEgpuRecoverMinAttemptIntervalMs   H2 rate limit (30000 ms)
 *   NVreg_TbEgpuRecoverSurrenderResetSec      H1 idle reset (300 s)
 *   NVreg_TbEgpuRecoverTestForceTrigger       Phase 3 test override (0)
 *
 * Naming note (vs the pre-refactor legacy code): this state machine was
 * developed as "Lever M-recover". The lever name was a process-history
 * artefact; symbols here describe what the code does — recovery — not
 * how it was discovered.
 */

#ifndef _NV_TB_EGPU_RECOVER_H_
#define _NV_TB_EGPU_RECOVER_H_

#include <linux/atomic.h>
#include <linux/pci.h>      /* pci_ers_result_t */
#include <linux/types.h>
#include <linux/workqueue.h>

struct nv_linux_state_s;
typedef struct nv_linux_state_s nv_linux_state_t;
struct pci_dev;

/*
 * Persistent kill-switch file. If present and content begins with '0',
 * the file overrides NVreg_TbEgpuRecoverEnable=1 (H3). This is the
 * panic-mode disable when recovery itself misbehaves; udev / userspace
 * helpers maintain it.
 */
#define TB_EGPU_RECOVER_KILLSWITCH_PATH \
    "/var/lib/tb-egpu/recover-killswitch"

/*
 * Per-device recovery state. Allocated in tb_egpu_recover_init when
 * NVreg_TbEgpuRecoverEnable=1; nvl->recover stays NULL otherwise.
 */
struct tb_egpu_recover_state
{
    /*
     * Recovery action serialised via this work item. Triggered from
     * either the post-rmInit-FAIL hook or the AER NEED_RESET path.
     */
    struct work_struct  reset_work;

    /*
     * pdev refcounted at trigger via pci_dev_get; released at handler
     * completion via pci_dev_put. Owned exclusively by the trigger
     * (writer) under the in_progress guard and the work handler
     * (reader + clearer). See the in_progress xchg ordering note in
     * tb_egpu_recover_trigger_post_rminit_fail().
     */
    struct pci_dev     *pdev_for_work;

    /*
     * Re-entry guard. Set via atomic_xchg(1) at trigger; cleared at
     * end of handler. Prevents recovery storms if multiple triggers
     * fire simultaneously.
     */
    atomic_t            in_progress;

    /* Cumulative counters for visibility. Read-only via sysfs. */
    atomic_t            fire_count;       /* schedule_work invocations */
    atomic_t            success_count;    /* recoveries that completed */
    atomic_t            surrender_count;  /* H1 / hard failure counts  */

    /*
     * Last fire timestamp (jiffies). Doubles as the H2 rate-limit
     * reference and the H1 burst-boundary timer
     * (NVreg_TbEgpuRecoverSurrenderResetSec).
     */
    unsigned long       last_fire_jiffies;

    /*
     * Per-pdev attempt counter — current burst. Per design § H1, this
     * resets to 0 ONLY at verified end-to-end recovery, i.e. when
     * nv_start_device observes rm_init_adapter success
     * (tb_egpu_recover_record_post_rminit_ok). Resetting any earlier
     * (e.g. at slot_reset_resume) leaves the H1 gate unreachable in
     * real-world recovery storms — a successful bus reset followed by
     * another failed rm_init_adapter is still a failed recovery for H1
     * accounting purposes.
     */
    atomic_t            attempt_count;
};

/* Module-parameter externs — defined in nv-tb-egpu-recover.c, referenced
 * from nv-pci.c (H4 error_detected gates) and this header. */
extern unsigned int NVreg_TbEgpuRecoverEnable;
extern unsigned int NVreg_TbEgpuRecoverMaxAttempts;
extern unsigned int NVreg_TbEgpuRecoverResetSettleMs;
extern unsigned int NVreg_TbEgpuRecoverMinAttemptIntervalMs;
extern unsigned int NVreg_TbEgpuRecoverSurrenderResetSec;
extern unsigned int NVreg_TbEgpuRecoverTestForceTrigger;

/* Lifecycle */
int  tb_egpu_recover_init(nv_linux_state_t *nvl);
void tb_egpu_recover_stop(nv_linux_state_t *nvl);

/*
 * Probe-time WPR2-stuck detection.
 *
 * Reads the GSP MMU WPR2 status register (BAR0 + 0x88a828). If bits
 * [31:4] are non-zero, WPR2 was left up by a prior boot cycle and GSP
 * boot will fail in the subsequent rm_init_adapter call.
 *
 * Detection-only (no recovery action): increments fire_count and emits
 * an NV_DBG_ERRORS marker. Caller-side counterpart at probe time;
 * complements tb_egpu_recover_trigger_post_rminit_fail() which is the
 * load-bearing trigger path. Returns true iff WPR2 detected up.
 */
bool tb_egpu_recover_check_wpr2_at_probe(nv_linux_state_t *nvl,
                                         u64 bar0_phys_addr);

/*
 * Post-rmInit-FAIL trigger. Called from nv_start_device() after
 * rm_init_adapter returns failure. Reads WPR2; if non-zero (and gates
 * pass) schedules an asynchronous upstream-bridge bus reset on the
 * kernel-global workqueue. Returns 0 in all cases — the caller's
 * existing failure path runs unchanged.
 */
int  tb_egpu_recover_trigger_post_rminit_fail(nv_linux_state_t *nvl);

/*
 * Post-rmInit-OK hook. Called from nv_start_device() after
 * rm_init_adapter returns success — the verified end-to-end recovery
 * moment per design § H1. Resets attempt_count to 0 so the next burst
 * starts fresh. Idempotent on cold boot (counter is already 0).
 */
void tb_egpu_recover_record_post_rminit_ok(nv_linux_state_t *nvl);

/*
 * Emit TB_EGPU_GPU_STATE=<state> uevent on the pdev's kobject. State
 * is one of "READY", "RECOVERING", "PERMANENT_FAIL". Userspace
 * subscribers (udev rules, services) react to recovery lifecycle.
 */
void tb_egpu_recover_emit_uevent(struct pci_dev *pdev, const char *state);

/*
 * pci_error_handlers slot_reset / resume bodies. Called by the thin
 * dispatchers in nv-pci.c (nv_pci_slot_reset / nv_pci_resume) and also
 * directly from the work handler after pci_reset_bus succeeds (since
 * pci_reset_bus does not itself dispatch err_handlers callbacks).
 */
pci_ers_result_t tb_egpu_recover_slot_reset(struct pci_dev *pdev);
void             tb_egpu_recover_slot_reset_resume(struct pci_dev *pdev);

/*
 * Pre-schedule gate result. Used by both the post-rmInit-FAIL trigger
 * (nv-tb-egpu-recover.c) and the AER error_detected callback (nv-pci.c).
 */
enum tb_egpu_recover_gate {
    TB_EGPU_RECOVER_GATE_OK = 0,
    TB_EGPU_RECOVER_GATE_DISABLED,
    TB_EGPU_RECOVER_GATE_RATE_LIMITED,
    TB_EGPU_RECOVER_GATE_SURRENDER,
};

/*
 * Single source of truth for the H1/H2/Enable gate decision. Both
 * the post-rmInit-FAIL trigger and the AER error_detected callback
 * call this.
 *
 * Side effects (callers MUST be aware):
 *   - At the H1 burst boundary (idle > SurrenderResetSec) attempt_count
 *     is reset to 0 here.
 *   - On GATE_OK: atomic_inc_return(&attempt_count). The caller then
 *     owns the rest of the schedule (set last_fire_jiffies, fire_count,
 *     in_progress, pdev_for_work, schedule_work, RECOVERING uevent).
 *   - On GATE_SURRENDER: atomic_inc(&surrender_count) and a
 *     PERMANENT_FAIL uevent against pdev. The attempt_count was already
 *     incremented (kept monotonic across surrenders).
 *
 * reason_out (optional) receives a short static string for logging.
 */
enum tb_egpu_recover_gate tb_egpu_recover_pre_schedule_gates(
    struct tb_egpu_recover_state *st,
    struct pci_dev *pdev,
    const char **reason_out);

#endif /* _NV_TB_EGPU_RECOVER_H_ */
