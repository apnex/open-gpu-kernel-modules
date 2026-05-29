/*
 * SPDX-FileCopyrightText: nvidia-driver-injector contributors
 * SPDX-License-Identifier: MIT
 *
 * nv-tb-egpu-metrics.c — Thunderbolt-eGPU F40b observability surface (addon A8).
 *
 * See nv-tb-egpu-metrics.h for the design rationale (including why A8 v2 uses
 * sysfs_create_group rather than the v1 pci_driver.driver.dev_groups, which the
 * kernel's __pci_register_driver clobbers to NULL).
 *
 * The backing metrics struct is module-global single-instance (single-eGPU
 * deployment posture). The attribute group attaches per-device on the PCI
 * device kobj; the storage is shared. tb_egpu_metrics_init resets the metrics
 * to a clean generation before publishing the group, so an unbind-rebind
 * (DaemonSet pod restart) does not surface stale counters from the prior bind.
 */

#include "nv-tb-egpu-metrics.h"
#include "nv-linux.h"
#include "nv-pci-types.h"

#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/ktime.h>
#include <linux/pci.h>
#include <linux/sysfs.h>
#include <linux/types.h>

/* -----------------------------------------------------------------------
 * Module-global metrics state
 * --------------------------------------------------------------------- */

enum nv_tb_egpu_state {
    NV_TB_EGPU_STATE_HEALTHY        = 0,
    NV_TB_EGPU_STATE_RECOVERING     = 1,
    NV_TB_EGPU_STATE_LOST_TEMPORARY = 2,
    NV_TB_EGPU_STATE_LOST_PERMANENT = 3,
};

static struct {
    atomic_t   state;
    atomic_t   f40b_fires;
    atomic_t   recovery_count;
    atomic_t   recovery_failures;
    atomic64_t last_recovery_ns;
} nv_tb_egpu_metrics = {
    .state             = ATOMIC_INIT(NV_TB_EGPU_STATE_HEALTHY),
    .f40b_fires        = ATOMIC_INIT(0),
    .recovery_count    = ATOMIC_INIT(0),
    .recovery_failures = ATOMIC_INIT(0),
    .last_recovery_ns  = ATOMIC64_INIT(0),
};

static const char *nv_tb_egpu_state_str(int s)
{
    switch (s) {
    case NV_TB_EGPU_STATE_HEALTHY:        return "healthy";
    case NV_TB_EGPU_STATE_RECOVERING:     return "recovering";
    case NV_TB_EGPU_STATE_LOST_TEMPORARY: return "lost-temporary";
    case NV_TB_EGPU_STATE_LOST_PERMANENT: return "lost-permanent";
    default:                               return "unknown";
    }
}

/*
 * Reset all metrics to a clean generation. Called at the top of
 * tb_egpu_metrics_init (i.e. at each device bind) BEFORE the sysfs group is
 * published, so a DaemonSet pod restart / unbind-rebind starts from
 * state=healthy / counters=0 rather than re-exposing the prior bind's counters
 * (the metrics struct is module-static and otherwise never reset).
 */
static void nv_tb_egpu_metrics_reset(void)
{
    atomic_set(&nv_tb_egpu_metrics.state, NV_TB_EGPU_STATE_HEALTHY);
    atomic_set(&nv_tb_egpu_metrics.f40b_fires, 0);
    atomic_set(&nv_tb_egpu_metrics.recovery_count, 0);
    atomic_set(&nv_tb_egpu_metrics.recovery_failures, 0);
    atomic64_set(&nv_tb_egpu_metrics.last_recovery_ns, 0);
}

/* -----------------------------------------------------------------------
 * Write surface (hooks called from elsewhere in the driver)
 * --------------------------------------------------------------------- */

/*
 * Called by A6's bounded-wait OPEN-path wrapper on timeout. This is a genuine
 * "GPU failed to come up" event: the open syscall could not bring the chip up,
 * so the device is unusable. Count it AND transition state to lost-temporary.
 */
void nv_tb_egpu_f40b_fired(void)
{
    atomic_inc(&nv_tb_egpu_metrics.f40b_fires);
    atomic_set(&nv_tb_egpu_metrics.state, NV_TB_EGPU_STATE_LOST_TEMPORARY);
}

/*
 * Called by A7's bounded-wait SHUTDOWN-path wrapper on timeout. This fires
 * during teardown (rmmod, or a close-path nv_stop_device before persistence),
 * where rm_shutdown_adapter's MMIO times out structurally on this hardware (A7
 * Test A: every healthy rmmod). It is a teardown artifact, NOT a "GPU is now
 * unusable" event — a fresh bind works fine afterward. So count it (the
 * f40b_fires counter honestly covers both fire paths, per A7's contract) but
 * do NOT touch state: stranding state at lost-temporary after a normal bring-up
 * (which includes a pre-persistence close-path fire) is a false-negative health
 * signal that never self-heals (A9's recovery_succeeded is the only path back to
 * healthy, and A9 is not wired). Leaving state alone keeps it at the value the
 * reset-on-bind established (healthy) through a routine teardown.
 */
void nv_tb_egpu_f40b_fired_teardown(void)
{
    atomic_inc(&nv_tb_egpu_metrics.f40b_fires);
}

/* Reserved for A9 — increments recovery_count and resets state to healthy. */
void nv_tb_egpu_recovery_succeeded(void)
{
    atomic_inc(&nv_tb_egpu_metrics.recovery_count);
    atomic64_set(&nv_tb_egpu_metrics.last_recovery_ns, ktime_get_ns());
    atomic_set(&nv_tb_egpu_metrics.state, NV_TB_EGPU_STATE_HEALTHY);
}

/* Reserved for A9 — increments recovery_failures.  State transition (to
 * lost-temporary on retry or lost-permanent on surrender) follows policy. */
void nv_tb_egpu_recovery_failed(int permanent)
{
    atomic_inc(&nv_tb_egpu_metrics.recovery_failures);
    atomic_set(&nv_tb_egpu_metrics.state,
        permanent ? NV_TB_EGPU_STATE_LOST_PERMANENT
                  : NV_TB_EGPU_STATE_LOST_TEMPORARY);
}

/* -----------------------------------------------------------------------
 * sysfs show functions (read the module-global metrics directly)
 * --------------------------------------------------------------------- */

static ssize_t tb_egpu_state_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    return sysfs_emit(buf, "%s\n",
        nv_tb_egpu_state_str(atomic_read(&nv_tb_egpu_metrics.state)));
}
static DEVICE_ATTR_RO(tb_egpu_state);

/* Per-device (not module-global): the E1 eGPU classification that gates the A6
 * open-path and A7 shutdown-path bounded-wait wrappers. Resolved live from the
 * device's nv_state_t so the value is correct regardless of when it was set.
 * Visibility for: confirming A6/A7 are armed on this bind, and detecting the
 * case where a driver unbind/rebind re-probes WITHOUT re-establishing the
 * classification (which silently disengages the bounded-wait guards). A plain
 * read() — no chip touch — so it is safe to poll even on a wedge-prone chip. */
static ssize_t tb_egpu_is_external_show(struct device *dev,
                                        struct device_attribute *attr, char *buf)
{
    nv_linux_state_t *nvl = pci_get_drvdata(to_pci_dev(dev));
    nv_state_t *nv;

    if (nvl == NULL)
        return sysfs_emit(buf, "unknown\n");

    nv = NV_STATE_PTR(nvl);
    return sysfs_emit(buf, "%d\n", nv->is_external_gpu ? 1 : 0);
}
static DEVICE_ATTR_RO(tb_egpu_is_external);

static ssize_t tb_egpu_f40b_fires_show(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
    return sysfs_emit(buf, "%d\n", atomic_read(&nv_tb_egpu_metrics.f40b_fires));
}
static DEVICE_ATTR_RO(tb_egpu_f40b_fires);

static ssize_t tb_egpu_recovery_count_show(struct device *dev,
                                           struct device_attribute *attr, char *buf)
{
    return sysfs_emit(buf, "%d\n", atomic_read(&nv_tb_egpu_metrics.recovery_count));
}
static DEVICE_ATTR_RO(tb_egpu_recovery_count);

static ssize_t tb_egpu_recovery_failures_show(struct device *dev,
                                              struct device_attribute *attr, char *buf)
{
    return sysfs_emit(buf, "%d\n", atomic_read(&nv_tb_egpu_metrics.recovery_failures));
}
static DEVICE_ATTR_RO(tb_egpu_recovery_failures);

static ssize_t tb_egpu_last_recovery_ns_show(struct device *dev,
                                             struct device_attribute *attr, char *buf)
{
    return sysfs_emit(buf, "%llu\n",
        (unsigned long long)atomic64_read(&nv_tb_egpu_metrics.last_recovery_ns));
}
static DEVICE_ATTR_RO(tb_egpu_last_recovery_ns);

static struct attribute *tb_egpu_metrics_attrs[] = {
    &dev_attr_tb_egpu_state.attr,
    &dev_attr_tb_egpu_is_external.attr,
    &dev_attr_tb_egpu_f40b_fires.attr,
    &dev_attr_tb_egpu_recovery_count.attr,
    &dev_attr_tb_egpu_recovery_failures.attr,
    &dev_attr_tb_egpu_last_recovery_ns.attr,
    NULL,
};

static const struct attribute_group tb_egpu_metrics_attr_group = {
    .attrs = tb_egpu_metrics_attrs,
};

/* -----------------------------------------------------------------------
 * Lifecycle (mirrors tb_egpu_recover_init / _stop and tb_egpu_qwd_init / _stop)
 * --------------------------------------------------------------------- */

int tb_egpu_metrics_init(nv_linux_state_t *nvl)
{
    nv_state_t *nv;

    if (!nvl)
        return 0;

    nv = NV_STATE_PTR(nvl);

    /* Start each bind generation clean (graft: reset-on-bind). The metrics
     * struct is module-static, so without this an unbind-rebind would surface
     * stale counters from the prior bind. */
    nv_tb_egpu_metrics_reset();

    /* Publish sysfs surface. Failure is non-fatal — the driver still binds,
     * just without userspace-visible counters. Mirrors A2/A3. */
    if (nvl->pci_dev)
    {
        int rc = sysfs_create_group(&nvl->pci_dev->dev.kobj,
                                    &tb_egpu_metrics_attr_group);
        if (rc != 0)
            NV_DEV_PRINTF(NV_DBG_INFO, nv,
                "tb_egpu metrics: sysfs_create_group failed: %d\n", rc);
    }

    return 0;
}

void tb_egpu_metrics_stop(nv_linux_state_t *nvl)
{
    if (!nvl)
        return;

    /* Remove sysfs surface. Benign no-op if it was never created. No backing
     * state to free — the metrics struct is module-static. */
    if (nvl->pci_dev)
    {
        sysfs_remove_group(&nvl->pci_dev->dev.kobj,
                           &tb_egpu_metrics_attr_group);
    }
}
