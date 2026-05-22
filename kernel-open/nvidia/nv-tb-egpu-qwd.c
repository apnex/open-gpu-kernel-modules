/*
 * SPDX-FileCopyrightText: nvidia-driver-injector contributors
 * SPDX-License-Identifier: MIT
 *
 * nv-tb-egpu-qwd.c — Thunderbolt-eGPU Q-watchdog (addon layer A2).
 *
 * Per-device kthread that periodically reads NV_PMC_BOOT_0 (offset 0 of
 * BAR0) via direct volatile MMIO. On 0xFFFFFFFF, declares the GPU
 * disconnected via os_pci_set_disconnected — same kernel-side
 * propagation as Q-active, but driven by an active heartbeat rather
 * than waiting for an ioctl-path MMIO read.
 *
 * Why an active probe: the 2026-05-05 Mode B silent freeze
 * (loop-2026-05-05-165029) wedged in the DMA-upload path (model load
 * via UVM), where no MMIO reads fire from userspace context. Q-active
 * stayed silent. With Q-watchdog, the bus drop is detected within
 * ~NVreg_TbEgpuQwdIntervalMs of failure regardless of which subsystem
 * stalled.
 *
 * Scope (v1):
 *   - Active heartbeat at fixed interval (default 200 ms)
 *   - On dead-bus detection: log marker, increment counter, mark
 *     disconnected via os_pci_set_disconnected — Q-passive then
 *     short-circuits all subsequent osDevReadReg* calls
 *   - Per-device counters (cycles, detections) for A/B characterisation
 *   - Persistent S3 detection state (jiffies, pmc_boot_0, AER snapshot)
 *     exposed via sysfs for cross-boot post-mortems
 *   - Module-param kill switch (NVreg_TbEgpuQwdEnable=0)
 *
 * Out of scope for v1 (future enhancements):
 *   - Auto-FLR on detection (handled by cluster P2 recovery state machine)
 *   - kobject_uevent emission (couples with the watchdog daemon)
 *   - DPM-aware backoff — not needed: this driver runs with
 *     NVreg_DynamicPowerManagement=0 forced via etc/modprobe.d, plus
 *     udev keeps power/control=on and d3cold_allowed=0, so the device
 *     stays in D0 and active MMIO is always safe.
 *
 * Sovereignty / layer (per docs/architecture-and-modularity.md):
 *   L1 (NVIDIA fork) — justified because the kthread needs:
 *     - access to nv_state_t internals (regs->map)
 *     - lifecycle binding to nvidia.ko probe/remove
 *     - call into os_pci_set_disconnected (project-added kernel-open API)
 *
 * Heisenbug acknowledgement (per memory feedback_observability_perturbs_bug):
 *   A 5 Hz active MMIO read is more perturbing than passive bpftrace.
 *   The runtime kill-switch (NVreg_TbEgpuQwdEnable) and per-device
 *   counters exist precisely so we can A/B characterise whether the
 *   watchdog itself materially changes the bug rate. The 2026-05-05
 *   freeze (which fired without heavy observability) suggests the bug
 *   is not purely Heisenbug-driven; we expect Q-watchdog to provide
 *   real signal rather than perturb the bug away.
 *
 * Cross-cluster dependency: the S3 AER snapshot (last_aer) is populated
 * by tb_egpu_dump_aer_trigger_event() in nv-tb-egpu-pcie.c (addon A1).
 * A3 (recovery state machine) patches the detect path below to add the
 * one-line call. Until A3 is applied, last_aer.valid stays 0 and the
 * sysfs reader emits a "(no detection event yet)" placeholder. The rest
 * of Q-watchdog (jiffies, pmc_boot_0, counters, kthread) functions
 * standalone on top of A1.
 */

#include "os-interface.h"
#include "nv-linux.h"
#include "nv-tb-egpu-qwd.h"

#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/moduleparam.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/sysfs.h>

/* Module parameters */

static unsigned int NVreg_TbEgpuQwdEnable = 1;
module_param(NVreg_TbEgpuQwdEnable, uint, 0644);
MODULE_PARM_DESC(NVreg_TbEgpuQwdEnable,
    "tb_egpu: enable Q-watchdog kthread heartbeat MMIO probe "
    "(1 = on (default), 0 = off). Runtime-toggleable; takes effect "
    "next probe cycle.");

static unsigned int NVreg_TbEgpuQwdIntervalMs = 200;
module_param(NVreg_TbEgpuQwdIntervalMs, uint, 0644);
MODULE_PARM_DESC(NVreg_TbEgpuQwdIntervalMs,
    "tb_egpu: Q-watchdog probe period in milliseconds "
    "(default 200, clamped to [10, 60000]).");

#define TB_EGPU_QWD_MIN_INTERVAL_MS     10U
#define TB_EGPU_QWD_MAX_INTERVAL_MS     60000U

/*
 * PMC_BOOT_0 lives at BAR0 offset 0 across all NVIDIA architectures.
 * Hardcoded here to keep this file independent of the RM-side
 * NV_PMC_BOOT_0 constant from src/common/inc/swref/published/nv_ref.h.
 */
#define TB_EGPU_QWD_PMC_BOOT_0_OFFSET   0u

/*
 * Dead-bus signature: PCIe completes config/MMIO reads for a fallen-off
 * device with all-1s after the hardware completion timeout. Mirrors the
 * RM-side TB_EGPU_DEAD_BUS_U32 from nv-tb-egpu.h; redefined here because
 * kernel-open does not include the RM tree's header search path.
 */
#define TB_EGPU_QWD_DEAD_BUS_VALUE      0xFFFFFFFFu

static int tb_egpu_qwd_thread(void *data)
{
    nv_linux_state_t        *nvl = (nv_linux_state_t *)data;
    nv_state_t              *nv  = NV_STATE_PTR(nvl);
    struct tb_egpu_qwd      *qwd = nvl->qwd;
    volatile NvU32          *regs32;
    NvU32                    boot_0;
    unsigned int             interval_ms;
    int                      detected_logged = 0;

    NV_DEV_PRINTF(NV_DBG_INFO, nv,
        "tb_egpu: qwd kthread started (interval=%ums, enable=%u)\n",
        NVreg_TbEgpuQwdIntervalMs, NVreg_TbEgpuQwdEnable);

    while (!kthread_should_stop())
    {
        /* Re-read interval each cycle so runtime tuning takes effect. */
        interval_ms = NVreg_TbEgpuQwdIntervalMs;
        if (interval_ms < TB_EGPU_QWD_MIN_INTERVAL_MS)
            interval_ms = TB_EGPU_QWD_MIN_INTERVAL_MS;
        if (interval_ms > TB_EGPU_QWD_MAX_INTERVAL_MS)
            interval_ms = TB_EGPU_QWD_MAX_INTERVAL_MS;

        msleep_interruptible(interval_ms);

        if (kthread_should_stop())
            break;

        /* Runtime kill-switch. */
        if (!NVreg_TbEgpuQwdEnable)
            continue;

        /*
         * Defensive: skip if regs torn down (early probe / late remove
         * race window). kthread_stop in tb_egpu_qwd_stop should prevent
         * this, but belt and suspenders.
         */
        if (!nv || !nv->regs || !nv->regs->map)
            continue;

        /*
         * Skip if already declared disconnected — Q-passive will
         * short-circuit subsequent userspace reads, no need to log
         * additional Q-watchdog detections during the same episode.
         */
        if (os_pci_is_disconnected(nv->handle))
            continue;

        /*
         * Active heartbeat: read PMC_BOOT_0 directly via volatile MMIO.
         * Bypasses osDevReadReg032 (the Q-active wrapper) deliberately
         * so Q-watchdog firings are cleanly attributable in dmesg vs
         * Q-active firings. PMC_BOOT_0 returns the chip ID on a healthy
         * GPU and 0xFFFFFFFF on a dead bus — a single read suffices for
         * the determination. READ_ONCE guarantees the compiler emits a
         * single 32-bit load without tearing or reordering.
         */
        regs32 = (volatile NvU32 *)nv->regs->map;
        boot_0 = READ_ONCE(regs32[TB_EGPU_QWD_PMC_BOOT_0_OFFSET]);
        atomic_inc(&qwd->cycles);

        if (boot_0 == TB_EGPU_QWD_DEAD_BUS_VALUE)
        {
            atomic_inc(&qwd->detections);

            if (!detected_logged)
            {
                detected_logged = 1;
                NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
                    "tb_egpu: qwd DETECTED dead bus "
                    "(PMC_BOOT_0=0x%08x after %d cycles).\n"
                    "  action: os_pci_set_disconnected called; "
                    "subsequent ioctl-path MMIO reads will short-circuit "
                    "via Q-passive.\n",
                    boot_0, atomic_read(&qwd->cycles));

                /*
                 * S3 persistent detection state — populate inside the
                 * latch so it captures the first detection of an
                 * episode. The AER snapshot (qwd->last_aer) is left
                 * untouched here; addon A3 patches in the call to
                 * tb_egpu_dump_aer_trigger_event() at this site to
                 * populate it.
                 */
                qwd->last_detection_jiffies = jiffies;
                qwd->last_pmc_boot_0        = boot_0;
            }

            os_pci_set_disconnected(nv->handle);
            /*
             * Keep looping: counters keep ticking; if the disconnect
             * ever clears we'll re-fire and re-log.
             */
        }
        else
        {
            /* Healthy reading; reset latch so a future episode logs again. */
            detected_logged = 0;
        }
    }

    NV_DEV_PRINTF(NV_DBG_INFO, nv,
        "tb_egpu: qwd kthread stopped "
        "(cycles=%d detections=%d)\n",
        atomic_read(&qwd->cycles), atomic_read(&qwd->detections));

    return 0;
}

/* sysfs surface — per-attr layout, registered via attribute_group. */

static ssize_t tb_egpu_qwd_cycles_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    nv_linux_state_t *nvl = pci_get_drvdata(pdev);
    if (!nvl || !nvl->qwd)
        return scnprintf(buf, PAGE_SIZE, "0\n");
    return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&nvl->qwd->cycles));
}

static ssize_t tb_egpu_qwd_detections_show(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    nv_linux_state_t *nvl = pci_get_drvdata(pdev);
    if (!nvl || !nvl->qwd)
        return scnprintf(buf, PAGE_SIZE, "0\n");
    return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&nvl->qwd->detections));
}

static ssize_t tb_egpu_qwd_last_detection_jiffies_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    nv_linux_state_t *nvl = pci_get_drvdata(pdev);
    if (!nvl || !nvl->qwd)
        return scnprintf(buf, PAGE_SIZE, "0\n");
    return scnprintf(buf, PAGE_SIZE, "%lu\n", nvl->qwd->last_detection_jiffies);
}

static ssize_t tb_egpu_qwd_last_pmc_boot_0_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    nv_linux_state_t *nvl = pci_get_drvdata(pdev);
    if (!nvl || !nvl->qwd)
        return scnprintf(buf, PAGE_SIZE, "0x00000000\n");
    return scnprintf(buf, PAGE_SIZE, "0x%08x\n", nvl->qwd->last_pmc_boot_0);
}

static ssize_t tb_egpu_qwd_last_aer_summary_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    struct pci_dev *pdev = to_pci_dev(dev);
    nv_linux_state_t *nvl = pci_get_drvdata(pdev);
    struct tb_egpu_qwd_aer_snapshot *s;

    if (!nvl || !nvl->qwd)
        return scnprintf(buf, PAGE_SIZE, "(no qwd state)\n");
    s = &nvl->qwd->last_aer;
    if (!s->valid)
        return scnprintf(buf, PAGE_SIZE,
            "(no detection event yet — qwd has run %d cycles)\n",
            atomic_read(&nvl->qwd->cycles));
    return scnprintf(buf, PAGE_SIZE,
        "GPU_AER_UESta=0x%08x  CESta=0x%08x\n"
        "Br_AER_UESta=0x%08x   CESta=0x%08x\n"
        "Root_AER_UESta=0x%08x CESta=0x%08x\n"
        "Root_RootSta=0x%08x   DPC_Status=0x%04x\n"
        "captured_at_jiffies=%lu pmc_boot_0=0x%08x\n",
        s->gpu_aer_uesta, s->gpu_aer_cesta,
        s->br_aer_uesta, s->br_aer_cesta,
        s->root_aer_uesta, s->root_aer_cesta,
        s->root_rootsta, s->dpc_status,
        nvl->qwd->last_detection_jiffies, nvl->qwd->last_pmc_boot_0);
}

static DEVICE_ATTR(tb_egpu_qwd_cycles, 0444,
                   tb_egpu_qwd_cycles_show, NULL);
static DEVICE_ATTR(tb_egpu_qwd_detections, 0444,
                   tb_egpu_qwd_detections_show, NULL);
static DEVICE_ATTR(tb_egpu_qwd_last_detection_jiffies, 0444,
                   tb_egpu_qwd_last_detection_jiffies_show, NULL);
static DEVICE_ATTR(tb_egpu_qwd_last_pmc_boot_0, 0444,
                   tb_egpu_qwd_last_pmc_boot_0_show, NULL);
static DEVICE_ATTR(tb_egpu_qwd_last_aer_summary, 0444,
                   tb_egpu_qwd_last_aer_summary_show, NULL);

static struct attribute *tb_egpu_qwd_attrs[] = {
    &dev_attr_tb_egpu_qwd_cycles.attr,
    &dev_attr_tb_egpu_qwd_detections.attr,
    &dev_attr_tb_egpu_qwd_last_detection_jiffies.attr,
    &dev_attr_tb_egpu_qwd_last_pmc_boot_0.attr,
    &dev_attr_tb_egpu_qwd_last_aer_summary.attr,
    NULL,
};

static const struct attribute_group tb_egpu_qwd_attr_group = {
    .attrs = tb_egpu_qwd_attrs,
};

int tb_egpu_qwd_init(nv_linux_state_t *nvl)
{
    struct tb_egpu_qwd     *qwd;
    nv_state_t             *nv = NV_STATE_PTR(nvl);

    if (!NVreg_TbEgpuQwdEnable)
    {
        NV_DEV_PRINTF(NV_DBG_INFO, nv,
            "tb_egpu: qwd disabled at module load "
            "(NVreg_TbEgpuQwdEnable=0); kthread not spawned\n");
        nvl->qwd = NULL;
        return 0;
    }

    qwd = kzalloc(sizeof(*qwd), GFP_KERNEL);
    if (!qwd)
    {
        NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
            "tb_egpu: qwd kzalloc failed; "
            "continuing without watchdog\n");
        return -ENOMEM;
    }

    atomic_set(&qwd->cycles, 0);
    atomic_set(&qwd->detections, 0);

    qwd->thread = kthread_run(tb_egpu_qwd_thread, nvl,
                              "tb-egpu-qwd-%02x%02x",
                              nv->pci_info.bus, nv->pci_info.slot);
    if (IS_ERR(qwd->thread))
    {
        int err = PTR_ERR(qwd->thread);
        NV_DEV_PRINTF(NV_DBG_ERRORS, nv,
            "tb_egpu: qwd kthread_run failed: %d; "
            "continuing without watchdog\n", err);
        kfree(qwd);
        nvl->qwd = NULL;
        return err;
    }

    nvl->qwd = qwd;

    /*
     * Publish sysfs surface. Failure is non-fatal — kthread still runs,
     * just without userspace-visible counters.
     */
    if (nvl->pci_dev)
    {
        int rc = sysfs_create_group(&nvl->pci_dev->dev.kobj,
                                    &tb_egpu_qwd_attr_group);
        if (rc != 0)
            NV_DEV_PRINTF(NV_DBG_INFO, nv,
                "tb_egpu: qwd sysfs_create_group failed: %d\n", rc);
    }

    return 0;
}

void tb_egpu_qwd_stop(nv_linux_state_t *nvl)
{
    struct tb_egpu_qwd *qwd;

    if (!nvl)
        return;

    /* Remove sysfs surface before kthread teardown. */
    if (nvl->pci_dev)
    {
        sysfs_remove_group(&nvl->pci_dev->dev.kobj,
                           &tb_egpu_qwd_attr_group);
    }

    qwd = nvl->qwd;
    if (!qwd)
        return;

    if (qwd->thread)
    {
        /*
         * kthread_stop blocks until the thread observes
         * kthread_should_stop and returns from its current sleep —
         * bounded by interval_ms (max 60s).
         */
        kthread_stop(qwd->thread);
        qwd->thread = NULL;
    }

    kfree(qwd);
    nvl->qwd = NULL;
}
