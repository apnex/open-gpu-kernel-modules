/*
 * SPDX-FileCopyrightText: nvidia-driver-injector contributors
 * SPDX-License-Identifier: MIT
 *
 * nv-tb-egpu-recover.c — Thunderbolt-eGPU recovery state machine (addon A3).
 *
 * In-driver recovery for two failure paths:
 *
 *   1. Post-rmInit-FAIL with WPR2 stuck. Trigger function reads the WPR2
 *      status register at BAR0+0x88a828 immediately after rm_init_adapter
 *      fails; on a stuck WPR2 it schedules a workqueue that runs
 *      pci_reset_bus() on the upstream Thunderbolt bridge. The work
 *      handler also dispatches our slot_reset / resume helpers
 *      explicitly, since pci_reset_bus does NOT itself fire
 *      err_handlers callbacks.
 *
 *   2. Runtime AER. nv_pci_error_detected runs the same H1/H2/H3 gates
 *      and returns PCI_ERS_RESULT_NEED_RESET on success; the kernel
 *      drives the bus reset and dispatches our slot_reset / resume
 *      helpers via the err_handlers struct registered in nv-pci.c.
 *
 * Both trigger paths funnel through tb_egpu_recover_pre_schedule_gates()
 * — a single source of truth for the gate logic so the two call sites
 * stay in lockstep.
 *
 * The shared PCIe/AER/WPR2 register-read primitives (tb_egpu_pcie_read_wpr2,
 * tb_egpu_dump_aer_trigger_event, the topology walker, the DPC / AER
 * extended-capability readers) live in the addon A1 foundation layer —
 * nv-tb-egpu-pcie.{c,h}. This file #includes nv-tb-egpu-pcie.h for them;
 * it does not redefine them.
 *
 * Sovereign layer: L1 (NVIDIA fork). Justified because the trigger and
 * the recovery callbacks need direct access to nv_state_t internals
 * (BAR0 physical address) and lifecycle-bound storage in nvl.
 *
 * DPM note: this driver runs with NVreg_DynamicPowerManagement=0 forced
 * via etc/modprobe.d, plus udev keeps power/control=on and
 * d3cold_allowed=0. The device stays in D0; all MMIO and PCI config
 * reads in this file are safe by construction.
 */

#include "nv-tb-egpu-recover.h"
#include "nv-tb-egpu-pcie.h"  /* A1: read_wpr2, dump_aer_trigger_event, ... */
#include "nv-linux.h"
#include "nv-pci-types.h"

#include <linux/atomic.h>
#include <linux/delay.h>      /* msleep */
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel_read_file.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

/* -----------------------------------------------------------------------
 * Module parameters
 *
 * TODO: flip NVreg_TbEgpuRecoverEnable default to 1 once production soak
 * passes (post-Phase 3, see docs/patch-refactor-status.md).
 * --------------------------------------------------------------------- */

unsigned int NVreg_TbEgpuRecoverEnable = 0;
module_param(NVreg_TbEgpuRecoverEnable, uint, 0644);
MODULE_PARM_DESC(NVreg_TbEgpuRecoverEnable,
    "tb_egpu: master enable for the in-driver recovery state machine "
    "(0=off default, 1=engage). The killswitch file at "
    "/var/lib/tb-egpu/recover-killswitch overrides to 0 when present "
    "and content begins with '0'.");

unsigned int NVreg_TbEgpuRecoverMaxAttempts = 3;
module_param(NVreg_TbEgpuRecoverMaxAttempts, uint, 0644);
MODULE_PARM_DESC(NVreg_TbEgpuRecoverMaxAttempts,
    "tb_egpu recover (H1): max bus-reset attempts per burst before "
    "surrendering (default 3). Counter resets on a verified end-to-end "
    "recovery (post-rmInit-OK) or after NVreg_TbEgpuRecoverSurrenderResetSec "
    "of idle.");

unsigned int NVreg_TbEgpuRecoverResetSettleMs = 500;
module_param(NVreg_TbEgpuRecoverResetSettleMs, uint, 0644);
MODULE_PARM_DESC(NVreg_TbEgpuRecoverResetSettleMs,
    "tb_egpu recover: settle delay (ms) after pci_reset_bus before "
    "verification (default 500).");

unsigned int NVreg_TbEgpuRecoverMinAttemptIntervalMs = 30000;
module_param(NVreg_TbEgpuRecoverMinAttemptIntervalMs, uint, 0644);
MODULE_PARM_DESC(NVreg_TbEgpuRecoverMinAttemptIntervalMs,
    "tb_egpu recover (H2): minimum interval (ms) between recovery "
    "attempts (default 30000 = 30s). Earlier triggers are rate-limited.");

unsigned int NVreg_TbEgpuRecoverSurrenderResetSec = 300;
module_param(NVreg_TbEgpuRecoverSurrenderResetSec, uint, 0644);
MODULE_PARM_DESC(NVreg_TbEgpuRecoverSurrenderResetSec,
    "tb_egpu recover (H1): seconds of idle after last_fire_jiffies "
    "before attempt_count resets to 0 (default 300 = 5min).");

unsigned int NVreg_TbEgpuRecoverTestForceTrigger = 0;
module_param(NVreg_TbEgpuRecoverTestForceTrigger, uint, 0644);
MODULE_PARM_DESC(NVreg_TbEgpuRecoverTestForceTrigger,
    "tb_egpu recover (test only): force the post-rmInit-FAIL trigger to "
    "fire even when WPR2 is clear (default 0).");

/* -----------------------------------------------------------------------
 * Kill-switch file (H3 persistence)
 * --------------------------------------------------------------------- */

/*
 * Read the persistent kill-switch file with a 16-byte hard cap.
 * kernel_read_file_from_path allocates via vmalloc; we vfree it.
 *
 * Returns:
 *    0 — file present, content begins '0' → kill switch ENGAGED
 *    1 — file present, content begins '1' → kill switch released
 *   -1 — file absent or unreadable → no override
 */
static int tb_egpu_recover_read_killswitch_file(void)
{
    void   *buf = NULL;
    size_t  file_size = 0;
    ssize_t bytes;
    int     result = -1;
    char    first;

    bytes = kernel_read_file_from_path(
        TB_EGPU_RECOVER_KILLSWITCH_PATH,
        0, &buf, 16, &file_size, READING_UNKNOWN);

    if (bytes < 0 || !buf)
        return -1;

    if (bytes > 0)
    {
        first = ((char *)buf)[0];
        if (first == '0')
            result = 0;
        else if (first == '1')
            result = 1;
    }

    vfree(buf);
    return result;
}

/*
 * Apply the kill-switch file override to NVreg_TbEgpuRecoverEnable.
 * Idempotent across multiple devices. Called once from recover_init.
 */
static int tb_egpu_recover_apply_killswitch_file(void)
{
    int file_state = tb_egpu_recover_read_killswitch_file();

    if (file_state == 0 && NVreg_TbEgpuRecoverEnable != 0)
    {
        nv_printf(NV_DBG_ERRORS,
            "tb_egpu recover: killswitch file engaged "
            "(%s=0); overriding NVreg_TbEgpuRecoverEnable to 0\n",
            TB_EGPU_RECOVER_KILLSWITCH_PATH);
        NVreg_TbEgpuRecoverEnable = 0;
    }
    return file_state;
}

/* -----------------------------------------------------------------------
 * uevent emission
 * --------------------------------------------------------------------- */

void tb_egpu_recover_emit_uevent(struct pci_dev *pdev, const char *state)
{
    char  envbuf[64];
    char *envp[2] = { envbuf, NULL };

    if (!pdev || !state)
        return;

    (void)scnprintf(envbuf, sizeof(envbuf), "TB_EGPU_GPU_STATE=%s", state);
    (void)kobject_uevent_env(&pdev->dev.kobj, KOBJ_CHANGE, envp);
}

/* -----------------------------------------------------------------------
 * Gate function — single source of truth for the recovery schedule
 * decision. Both the post-rmInit-FAIL trigger and the AER
 * error_detected callback call this.
 *
 * Semantics:
 *   - GATE_DISABLED       NVreg_TbEgpuRecoverEnable=0 or st=NULL
 *   - GATE_RATE_LIMITED   last fire was < MinAttemptIntervalMs ago (H2)
 *   - GATE_SURRENDER      attempt_count would exceed MaxAttempts (H1)
 *   - GATE_OK             schedule the recovery
 *
 * Side effects (callers MUST be aware):
 *   - If we cross the H1 burst boundary (idle > SurrenderResetSec) we
 *     reset attempt_count to 0 here so the next increment opens a fresh
 *     burst.
 *   - On GATE_OK we atomic_inc_return(&attempt_count). The caller owns
 *     the rest of the schedule (set last_fire_jiffies, fire_count,
 *     in_progress, pdev_for_work, schedule_work, RECOVERING uevent).
 *   - On GATE_SURRENDER we atomic_inc(&surrender_count) and emit a
 *     PERMANENT_FAIL uevent against pdev — the caller does NOT need to
 *     repeat that. We still atomic_inc_return on attempt_count to keep
 *     the counter monotonic across surrenders.
 *
 * reason_out (optional) receives a short static string suitable for
 * logging at the caller.
 * --------------------------------------------------------------------- */

enum tb_egpu_recover_gate tb_egpu_recover_pre_schedule_gates(
    struct tb_egpu_recover_state *st,
    struct pci_dev *pdev,
    const char **reason_out)
{
    unsigned long elapsed_ms;
    int attempts;

    if (!st || !NVreg_TbEgpuRecoverEnable)
    {
        if (reason_out)
            *reason_out = "disabled (NVreg_TbEgpuRecoverEnable=0 or state=NULL)";
        return TB_EGPU_RECOVER_GATE_DISABLED;
    }

    /* H1 burst boundary — idle long enough to start a fresh burst. */
    if (st->last_fire_jiffies != 0 &&
        time_after(jiffies,
                   st->last_fire_jiffies +
                   msecs_to_jiffies(NVreg_TbEgpuRecoverSurrenderResetSec * 1000U)))
    {
        atomic_set(&st->attempt_count, 0);
    }

    /* H2 rate-limit gate — cheaper than H1, check first. */
    if (st->last_fire_jiffies != 0)
    {
        elapsed_ms = jiffies_to_msecs(jiffies - st->last_fire_jiffies);
        if (elapsed_ms < NVreg_TbEgpuRecoverMinAttemptIntervalMs)
        {
            if (reason_out)
                *reason_out = "rate-limited (H2)";
            return TB_EGPU_RECOVER_GATE_RATE_LIMITED;
        }
    }

    /* H1 MaxAttempts gate. */
    attempts = atomic_inc_return(&st->attempt_count);
    if (attempts > (int)NVreg_TbEgpuRecoverMaxAttempts)
    {
        atomic_inc(&st->surrender_count);
        tb_egpu_recover_emit_uevent(pdev, "PERMANENT_FAIL");
        if (reason_out)
            *reason_out = "surrender (H1 MaxAttempts exhausted)";
        return TB_EGPU_RECOVER_GATE_SURRENDER;
    }

    if (reason_out)
        *reason_out = "scheduling bus reset";
    return TB_EGPU_RECOVER_GATE_OK;
}

/* -----------------------------------------------------------------------
 * Probe-time WPR2-stuck detection (informational only)
 *
 * The probe-time check is preserved from the legacy series as a
 * sanity-check counter incrementer — it's been empirically falsified
 * as the production trigger (boot-persistence hypothesis disproven
 * 2026-05-06), but it provides cheap fire-rate visibility and may
 * still fire on legitimate WPR2-stuck scenarios. Detection-only: never
 * schedules recovery from here.
 * --------------------------------------------------------------------- */

bool tb_egpu_recover_check_wpr2_at_probe(nv_linux_state_t *nvl,
                                         u64 bar0_phys_addr)
{
    nv_state_t *nv;
    u32         wpr2_raw = 0;
    u32         wpr2_val;
    int         rc;

    if (!nvl)
        return false;

    if (!NVreg_TbEgpuRecoverEnable || !nvl->recover)
        return false;

    nv = NV_STATE_PTR(nvl);

    rc = tb_egpu_pcie_read_wpr2(bar0_phys_addr, &wpr2_raw);
    if (rc != 0)
    {
        NV_DEV_PRINTF(NV_DBG_INFO, nv,
            "tb_egpu recover: probe WPR2 check — read failed (rc=%d); "
            "skipping (probe continues normally)\n", rc);
        return false;
    }

    wpr2_val = wpr2_raw & TB_EGPU_PCIE_WPR2_VAL_MASK;
    if (wpr2_val == 0)
    {
        NV_DEV_PRINTF(NV_DBG_INFO, nv,
            "tb_egpu recover: probe WPR2 clear (raw=0x%08x); "
            "GSP boot should proceed normally\n", wpr2_raw);
        return false;
    }

    atomic_inc(&nvl->recover->fire_count);
    nvl->recover->last_fire_jiffies = jiffies;

    NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
        "tb_egpu recover: probe WPR2 DETECTED up (raw=0x%08x val=0x%08x). "
        "Detection-only at probe; the load-bearing trigger is the "
        "post-rmInit-FAIL hook in nv_start_device. fire_count=%d\n",
        wpr2_raw, wpr2_val, atomic_read(&nvl->recover->fire_count));

    return true;
}

/* -----------------------------------------------------------------------
 * Work handler — runs pci_reset_bus on the upstream bridge, then
 * explicitly dispatches our slot_reset / resume helpers.
 *
 * Why the explicit dispatch: pci_reset_bus() resets the secondary bus
 * but does NOT invoke err_handlers->slot_reset / ->resume. Those fire
 * only when the kernel's AER subsystem drives recovery via the
 * NEED_RESET return path from error_detected. So the manual trigger
 * path (post-rmInit-FAIL hook + sysfs force_trigger) must call those
 * helpers itself to reach the same completion state — PMC_BOOT_0
 * verify, success_count++, READY uevent — that the AER path gets for
 * free. (The AER path is unchanged; pci_reset_bus is not called from
 * the work handler when the kernel drives recovery, only from the
 * manual trigger path.)
 *
 * pdev refcount: held by the trigger via pci_dev_get; we pci_dev_put
 * before returning.
 * --------------------------------------------------------------------- */

static void tb_egpu_recover_reset_work_handler(struct work_struct *work)
{
    struct tb_egpu_recover_state *st =
        container_of(work, struct tb_egpu_recover_state, reset_work);
    struct pci_dev *pdev   = st->pdev_for_work;
    struct pci_dev *bridge = NULL;
    int rc;

    if (!pdev)
    {
        nv_printf(NV_DBG_ERRORS,
            "tb_egpu recover: reset_work fired but pdev_for_work=NULL; "
            "recovery aborted\n");
        atomic_set(&st->in_progress, 0);
        return;
    }

    bridge = pci_upstream_bridge(pdev);
    if (!bridge)
    {
        nv_printf(NV_DBG_ERRORS,
            "tb_egpu recover: no upstream bridge for %s; cannot bus-reset; "
            "surrendering\n", pci_name(pdev));
        atomic_inc(&st->surrender_count);
        tb_egpu_recover_emit_uevent(pdev, "PERMANENT_FAIL");
        goto out_put;
    }

    nv_printf(NV_DBG_ERRORS,
        "tb_egpu recover: bus-reset starting on bridge %s "
        "(GPU=%s; attempt=%d/%u; settle=%ums)\n",
        pci_name(bridge), pci_name(pdev),
        atomic_read(&st->attempt_count), NVreg_TbEgpuRecoverMaxAttempts,
        NVreg_TbEgpuRecoverResetSettleMs);

    tb_egpu_recover_emit_uevent(pdev, "RECOVERING");

    /* Serialise vs the kernel's PCI rescan/remove paths. */
    pci_lock_rescan_remove();
    rc = pci_reset_bus(bridge);
    pci_unlock_rescan_remove();

    /* Settle before verification; gives the link time to retrain. */
    msleep(NVreg_TbEgpuRecoverResetSettleMs);

    if (rc != 0)
    {
        nv_printf(NV_DBG_ERRORS,
            "tb_egpu recover: pci_reset_bus(%s) FAILED rc=%d; "
            "emitting PERMANENT_FAIL\n", pci_name(bridge), rc);
        atomic_inc(&st->surrender_count);
        tb_egpu_recover_emit_uevent(pdev, "PERMANENT_FAIL");
        goto out_put;
    }

    nv_printf(NV_DBG_ERRORS,
        "tb_egpu recover: pci_reset_bus(%s) OK; dispatching slot_reset + "
        "resume helpers explicitly\n", pci_name(bridge));

    /*
     * Explicit dispatch — see file-level note. slot_reset reads
     * PMC_BOOT_0; on RECOVERED, resume increments success_count and
     * emits READY. On DISCONNECT, slot_reset itself handles the
     * surrender accounting + PERMANENT_FAIL.
     */
    {
        pci_ers_result_t rs = tb_egpu_recover_slot_reset(pdev);
        if (rs == PCI_ERS_RESULT_RECOVERED)
            tb_egpu_recover_slot_reset_resume(pdev);
    }

out_put:
    pci_dev_put(pdev);
    st->pdev_for_work = NULL;
    atomic_set(&st->in_progress, 0);
}

/* -----------------------------------------------------------------------
 * Post-rmInit-FAIL trigger
 *
 * Concurrency / ordering note (re: pdev_for_work race audit, refactor
 * brief item 6):
 *
 *   pdev_for_work is owned exclusively under the in_progress=1 guard.
 *   Order of operations on the trigger side:
 *     1. atomic_xchg(&in_progress, 1) returns the previous value.
 *     2. If previous was 1: another trigger is mid-flight; bail out
 *        without touching pdev_for_work.
 *     3. Otherwise we now own the slot. Run gates; on GATE_OK, write
 *        pdev_for_work = pci_dev_get(pdev) BEFORE schedule_work().
 *
 *   Order on the handler side:
 *     A. Read pdev_for_work (publishes-after the schedule_work
 *        store-release in step 3).
 *     B. ... do recovery ...
 *     C. pci_dev_put(pdev); st->pdev_for_work = NULL;
 *     D. atomic_set(&in_progress, 0).
 *
 *   Therefore: when atomic_xchg returns 0 here, the previous handler
 *   (if any) has already executed C → D, so pdev_for_work is NULL.
 *   The legacy "Defensive: stale pdev_for_work" branch (which dropped
 *   a refcount before re-arming) was dead-defensive code; removed. We
 *   keep a WARN_ON_ONCE as a tripwire in case the ordering ever
 *   regresses.
 * --------------------------------------------------------------------- */

int tb_egpu_recover_trigger_post_rminit_fail(nv_linux_state_t *nvl)
{
    struct tb_egpu_recover_state *st;
    nv_state_t                   *nv;
    struct pci_dev               *pdev;
    u64                           bar0_phys;
    u32                           wpr2_raw = 0;
    u32                           wpr2_val;
    bool                          wpr2_stuck;
    enum tb_egpu_recover_gate     gate;
    const char                   *reason = "?";

    if (!nvl)
        return 0;

    if (!NVreg_TbEgpuRecoverEnable || !nvl->recover)
        return 0;   /* fall back to existing failure path */

    st   = nvl->recover;
    nv   = NV_STATE_PTR(nvl);
    pdev = nvl->pci_dev;
    bar0_phys = nv->bars[NV_GPU_BAR_INDEX_REGS].cpu_address;

    if (!pdev || bar0_phys == 0)
        return 0;

    /* Step 1: read WPR2 (or honour TestForceTrigger). */
    (void)tb_egpu_pcie_read_wpr2(bar0_phys, &wpr2_raw);
    wpr2_val   = wpr2_raw & TB_EGPU_PCIE_WPR2_VAL_MASK;
    wpr2_stuck = (wpr2_val != 0);

    if (NVreg_TbEgpuRecoverTestForceTrigger && !wpr2_stuck)
    {
        NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
            "tb_egpu recover: TestForceTrigger=1 — forcing trigger even "
            "though WPR2 is clear (raw=0x%08x)\n", wpr2_raw);
        wpr2_stuck = true;
    }

    if (!wpr2_stuck)
    {
        NV_DEV_PRINTF(NV_DBG_INFO, nv,
            "tb_egpu recover: post-rmInit-FAIL but WPR2 clear "
            "(raw=0x%08x); not the WPR2-stuck failure mode; not triggering\n",
            wpr2_raw);
        return 0;
    }

    /* Step 2: re-entry guard (acquires exclusive ownership of pdev_for_work). */
    if (atomic_xchg(&st->in_progress, 1) != 0)
    {
        NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
            "tb_egpu recover: trigger fired but in_progress=1; previous "
            "attempt still running; skipping\n");
        return 0;
    }
    WARN_ON_ONCE(st->pdev_for_work != NULL);

    /* Step 3: H1/H2/Enable gates. */
    gate = tb_egpu_recover_pre_schedule_gates(st, pdev, &reason);
    switch (gate)
    {
        case TB_EGPU_RECOVER_GATE_OK:
            break;

        case TB_EGPU_RECOVER_GATE_DISABLED:
        case TB_EGPU_RECOVER_GATE_RATE_LIMITED:
            NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
                "tb_egpu recover: trigger gated (%s); deferring\n", reason);
            atomic_set(&st->in_progress, 0);
            return 0;

        case TB_EGPU_RECOVER_GATE_SURRENDER:
            NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
                "tb_egpu recover: trigger gated (%s); emitting PERMANENT_FAIL\n",
                reason);
            atomic_set(&st->in_progress, 0);
            return 0;
    }

    /* Step 4: schedule the work. */
    atomic_inc(&st->fire_count);
    st->last_fire_jiffies = jiffies;
    st->pdev_for_work     = pci_dev_get(pdev);

    NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
        "tb_egpu recover: scheduling recovery "
        "(attempt=%d/%u, WPR2=0x%08x, ForceTrigger=%u, fires=%d)\n",
        atomic_read(&st->attempt_count), NVreg_TbEgpuRecoverMaxAttempts,
        wpr2_raw, NVreg_TbEgpuRecoverTestForceTrigger,
        atomic_read(&st->fire_count));

    schedule_work(&st->reset_work);
    return 0;
}

/* -----------------------------------------------------------------------
 * slot_reset / resume / post-rmInit-OK
 * --------------------------------------------------------------------- */

pci_ers_result_t tb_egpu_recover_slot_reset(struct pci_dev *pdev)
{
    nv_linux_state_t *nvl;
    nv_state_t       *nv;
    void __iomem     *tmp_map;
    u64               bar0_phys;
    u32               pmc_boot_0 = 0xffffffffu;

    if (!pdev)
        return PCI_ERS_RESULT_DISCONNECT;

    nvl = pci_get_drvdata(pdev);
    if (!nvl)
        return PCI_ERS_RESULT_DISCONNECT;

    nv = NV_STATE_PTR(nvl);
    bar0_phys = nv->bars[NV_GPU_BAR_INDEX_REGS].cpu_address;
    if (bar0_phys == 0)
    {
        NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
            "tb_egpu recover: slot_reset — bar0_phys=0; cannot verify; "
            "DISCONNECT\n");
        return PCI_ERS_RESULT_DISCONNECT;
    }

    tmp_map = ioremap(bar0_phys, PAGE_SIZE);
    if (tmp_map)
    {
        pmc_boot_0 = ioread32(tmp_map);
        iounmap(tmp_map);
    }

    if (pmc_boot_0 == 0xffffffffu)
    {
        NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
            "tb_egpu recover: slot_reset — PMC_BOOT_0=0xffffffff "
            "(bus still down); DISCONNECT\n");
        if (nvl->recover)
        {
            atomic_inc(&nvl->recover->surrender_count);
            tb_egpu_recover_emit_uevent(pdev, "PERMANENT_FAIL");
        }
        return PCI_ERS_RESULT_DISCONNECT;
    }

    NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
        "tb_egpu recover: slot_reset — PMC_BOOT_0=0x%08x; RECOVERED\n",
        pmc_boot_0);
    return PCI_ERS_RESULT_RECOVERED;
}

/*
 * Resume callback. Bumps success_count and emits READY. The
 * attempt_count reset lives at tb_egpu_recover_record_post_rminit_ok
 * — slot_reset RECOVERED proves only that the bus is back, not that
 * GSP / rm_init_adapter will succeed on the next attempt. Per design
 * § H1 only verified end-to-end success clears the counter; resetting
 * here would leave the H1 gate unreachable in real-world storms.
 */
void tb_egpu_recover_slot_reset_resume(struct pci_dev *pdev)
{
    nv_linux_state_t *nvl;

    if (!pdev)
        return;

    nvl = pci_get_drvdata(pdev);
    if (!nvl || !nvl->recover)
        return;

    atomic_inc(&nvl->recover->success_count);

    nv_printf(NV_DBG_ERRORS,
        "tb_egpu recover: resume — success_count=%d, bus reset done; "
        "emitting READY (attempt_count cleared at next post-rmInit-OK)\n",
        atomic_read(&nvl->recover->success_count));

    tb_egpu_recover_emit_uevent(pdev, "READY");
}

void tb_egpu_recover_record_post_rminit_ok(nv_linux_state_t *nvl)
{
    int prev;

    if (!nvl || !nvl->recover)
        return;

    prev = atomic_read(&nvl->recover->attempt_count);
    if (prev > 0)
    {
        nv_printf(NV_DBG_ERRORS,
            "tb_egpu recover: post-rmInit-OK observed with attempt_count=%d; "
            "verified end-to-end recovery — resetting attempt_count to 0\n",
            prev);
    }
    atomic_set(&nvl->recover->attempt_count, 0);
}

/* -----------------------------------------------------------------------
 * sysfs surface
 * --------------------------------------------------------------------- */

static ssize_t tb_egpu_recover_fires_show(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    nv_linux_state_t *nvl = pci_get_drvdata(pdev);
    if (!nvl || !nvl->recover)
        return scnprintf(buf, PAGE_SIZE, "0\n");
    return scnprintf(buf, PAGE_SIZE, "%d\n",
                     atomic_read(&nvl->recover->fire_count));
}

static ssize_t tb_egpu_recover_successes_show(struct device *dev,
                                              struct device_attribute *attr,
                                              char *buf)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    nv_linux_state_t *nvl = pci_get_drvdata(pdev);
    if (!nvl || !nvl->recover)
        return scnprintf(buf, PAGE_SIZE, "0\n");
    return scnprintf(buf, PAGE_SIZE, "%d\n",
                     atomic_read(&nvl->recover->success_count));
}

static ssize_t tb_egpu_recover_surrenders_show(struct device *dev,
                                               struct device_attribute *attr,
                                               char *buf)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    nv_linux_state_t *nvl = pci_get_drvdata(pdev);
    if (!nvl || !nvl->recover)
        return scnprintf(buf, PAGE_SIZE, "0\n");
    return scnprintf(buf, PAGE_SIZE, "%d\n",
                     atomic_read(&nvl->recover->surrender_count));
}

static ssize_t tb_egpu_recover_last_fire_jiffies_show(struct device *dev,
                                                      struct device_attribute *attr,
                                                      char *buf)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    nv_linux_state_t *nvl = pci_get_drvdata(pdev);
    if (!nvl || !nvl->recover)
        return scnprintf(buf, PAGE_SIZE, "0\n");
    return scnprintf(buf, PAGE_SIZE, "%lu\n",
                     nvl->recover->last_fire_jiffies);
}

/*
 * Write-only force-trigger (Phase 3 test path). Calls the trigger
 * function directly. All gates apply — if Enable=0 it's a no-op; if
 * rate-limited it defers. With NVreg_TbEgpuRecoverTestForceTrigger=1
 * the trigger's WPR2-clear branch is overridden, exercising the full
 * recovery path on a healthy system without having to deliberately
 * fail rm_init_adapter.
 */
static ssize_t tb_egpu_recover_force_trigger_store(struct device *dev,
                                                   struct device_attribute *attr,
                                                   const char *buf, size_t count)
{
    struct pci_dev    *pdev = to_pci_dev(dev);
    nv_linux_state_t  *nvl;
    unsigned long      val;
    int                rc;

    if (!pdev)
        return -ENODEV;

    nvl = pci_get_drvdata(pdev);
    if (!nvl)
        return -ENODEV;

    rc = kstrtoul(buf, 0, &val);
    if (rc != 0)
        return rc;

    if (val == 0)
        return count;

    nv_printf(NV_DBG_ERRORS,
        "tb_egpu recover: sysfs force_trigger fired (val=%lu) — invoking "
        "trigger as if from post-rmInit-FAIL\n", val);
    (void)tb_egpu_recover_trigger_post_rminit_fail(nvl);

    return count;
}

static DEVICE_ATTR(tb_egpu_recover_fires, 0444,
                   tb_egpu_recover_fires_show, NULL);
static DEVICE_ATTR(tb_egpu_recover_successes, 0444,
                   tb_egpu_recover_successes_show, NULL);
static DEVICE_ATTR(tb_egpu_recover_surrenders, 0444,
                   tb_egpu_recover_surrenders_show, NULL);
static DEVICE_ATTR(tb_egpu_recover_last_fire_jiffies, 0444,
                   tb_egpu_recover_last_fire_jiffies_show, NULL);
static DEVICE_ATTR(tb_egpu_recover_force_trigger, 0200,
                   NULL, tb_egpu_recover_force_trigger_store);

static struct attribute *tb_egpu_recover_attrs[] = {
    &dev_attr_tb_egpu_recover_fires.attr,
    &dev_attr_tb_egpu_recover_successes.attr,
    &dev_attr_tb_egpu_recover_surrenders.attr,
    &dev_attr_tb_egpu_recover_last_fire_jiffies.attr,
    &dev_attr_tb_egpu_recover_force_trigger.attr,
    NULL,
};

static const struct attribute_group tb_egpu_recover_attr_group = {
    .attrs = tb_egpu_recover_attrs,
};

/* -----------------------------------------------------------------------
 * Lifecycle
 * --------------------------------------------------------------------- */

int tb_egpu_recover_init(nv_linux_state_t *nvl)
{
    struct tb_egpu_recover_state *st;
    nv_state_t                   *nv = NV_STATE_PTR(nvl);

    /* H3 — persistent kill-switch file can force Enable to 0 even if
     * the cmdline param says 1. Idempotent across multiple devices. */
    (void)tb_egpu_recover_apply_killswitch_file();

    if (!NVreg_TbEgpuRecoverEnable)
    {
        NV_DEV_PRINTF(NV_DBG_INFO, nv,
            "tb_egpu recover: disabled at module load "
            "(NVreg_TbEgpuRecoverEnable=0); state not allocated\n");
        nvl->recover = NULL;
        return 0;
    }

    st = kzalloc(sizeof(*st), GFP_KERNEL);
    if (!st)
    {
        NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
            "tb_egpu recover: kzalloc failed; continuing without "
            "in-driver recovery\n");
        return -ENOMEM;
    }

    INIT_WORK(&st->reset_work, tb_egpu_recover_reset_work_handler);
    atomic_set(&st->in_progress, 0);
    atomic_set(&st->fire_count, 0);
    atomic_set(&st->success_count, 0);
    atomic_set(&st->surrender_count, 0);
    atomic_set(&st->attempt_count, 0);
    st->last_fire_jiffies = 0;
    st->pdev_for_work     = NULL;

    nvl->recover = st;

    /* Publish sysfs surface. Failure is non-fatal — recovery still
     * runs, just without userspace-visible counters. */
    if (nvl->pci_dev)
    {
        int rc = sysfs_create_group(&nvl->pci_dev->dev.kobj,
                                    &tb_egpu_recover_attr_group);
        if (rc != 0)
            NV_DEV_PRINTF(NV_DBG_INFO, nv,
                "tb_egpu recover: sysfs_create_group failed: %d\n", rc);
    }

    NV_DEV_PRINTF(NV_DBG_INFO, nv,
        "tb_egpu recover: scaffolding initialised "
        "(MaxAttempts=%u, SettleMs=%u, MinIntervalMs=%u, ResetSec=%u)\n",
        NVreg_TbEgpuRecoverMaxAttempts,
        NVreg_TbEgpuRecoverResetSettleMs,
        NVreg_TbEgpuRecoverMinAttemptIntervalMs,
        NVreg_TbEgpuRecoverSurrenderResetSec);

    return 0;
}

void tb_egpu_recover_stop(nv_linux_state_t *nvl)
{
    struct tb_egpu_recover_state *st;

    if (!nvl)
        return;

    /* Remove sysfs surface before tearing down state. */
    if (nvl->pci_dev)
    {
        sysfs_remove_group(&nvl->pci_dev->dev.kobj,
                           &tb_egpu_recover_attr_group);
    }

    st = nvl->recover;
    if (!st)
        return;

    /* Drain any pending work item before freeing state. */
    cancel_work_sync(&st->reset_work);

    /* Drop any straggler refcount (handler normally pci_dev_puts). */
    if (st->pdev_for_work)
    {
        pci_dev_put(st->pdev_for_work);
        st->pdev_for_work = NULL;
    }

    kfree(st);
    nvl->recover = NULL;
}
