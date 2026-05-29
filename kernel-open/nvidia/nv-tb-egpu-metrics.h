/*
 * SPDX-FileCopyrightText: nvidia-driver-injector contributors
 * SPDX-License-Identifier: MIT
 *
 * nv-tb-egpu-metrics.h — Thunderbolt-eGPU F40b observability surface (addon A8).
 *
 * Exposes the F40b state machine's state + counters as read-only per-PCI-device
 * sysfs attributes, registered via sysfs_create_group on the device kobj at
 * probe time (mirroring the addon A2/A3 idiom). Five attributes appear under
 * /sys/bus/pci/devices/<DBDF>/ when nvidia.ko binds, and disappear on unbind:
 *
 *   tb_egpu_state              healthy | recovering | lost-temporary | lost-permanent
 *   tb_egpu_f40b_fires         monotonic counter of F40b timeout fires (A6)
 *   tb_egpu_recovery_count     monotonic counter of successful recoveries (A9)
 *   tb_egpu_recovery_failures  monotonic counter of failed recovery attempts (A9)
 *   tb_egpu_last_recovery_ns   ktime_get_ns() of last successful recovery (A9)
 *
 * Registration mechanism (A8 v2): explicit sysfs_create_group at probe /
 * sysfs_remove_group at remove, on &nvl->pci_dev->dev.kobj, exactly as A2's
 * Q-watchdog and A3's recovery state machine do. A8 v1 used
 * pci_driver.driver.dev_groups, which compiled and loaded all symbols but
 * produced ZERO sysfs paths at runtime: __pci_register_driver() executes
 * `drv->driver.dev_groups = drv->dev_groups;` (drivers/pci/pci-driver.c),
 * clobbering the inner .driver.dev_groups field A8 v1 set with the NULL outer
 * pci_driver.dev_groups. The sysfs_create_group approach is immune to that
 * clobber and is proven on this driver by A2/A3.
 *
 * Single-GPU assumption: the backing metrics struct is module-global
 * single-instance (one eGPU per host, this project's deployment posture). The
 * attribute group attaches per-device, but the backing storage is shared. On
 * each device bind the counters are reset to zero and state to 'healthy'
 * (tb_egpu_metrics_init), so a DaemonSet pod restart / unbind-rebind starts a
 * clean generation rather than re-exposing stale counters from the prior bind.
 * Multi-GPU support would require per-device storage via nv_linux_state_t.
 *
 * The three hooks below are the write surface, called from elsewhere in the
 * driver:
 *   nv_tb_egpu_f40b_fired()         — A6 open-path timeout: counter + state->lost (live)
 *   nv_tb_egpu_f40b_fired_teardown()— A7 shutdown-path timeout: counter only (live)
 *   nv_tb_egpu_recovery_succeeded() — A9's recovery state machine (forthcoming)
 *   nv_tb_egpu_recovery_failed()    — A9's recovery state machine (forthcoming)
 *
 * Telemetry tier: nominal. This TU emits no kernel-log lines; A6 (and A9) own
 * the human-readable markers. A8's job is the machine-readable surface.
 */

#ifndef _NV_TB_EGPU_METRICS_H_
#define _NV_TB_EGPU_METRICS_H_

struct nv_linux_state_s;
typedef struct nv_linux_state_s nv_linux_state_t;

/* Lifecycle — called from nv_pci_probe / nv_pci_remove_helper, mirroring the
 * A2/A3 init/stop pattern. Init resets metrics to a clean generation then
 * registers the sysfs group; stop removes the group. Both guard on nvl->pci_dev
 * and are non-fatal on sysfs failure (the driver still binds). */
int  tb_egpu_metrics_init(nv_linux_state_t *nvl);
void tb_egpu_metrics_stop(nv_linux_state_t *nvl);

/*
 * Write surface. Module-global single-instance metrics; safe to call from any
 * context (atomic ops only, no sleeping).
 *
 *   nv_tb_egpu_f40b_fired()         f40b_fires++; state -> lost-temporary.
 *                                   Called by A6's open-path timeout (genuine
 *                                   "GPU failed to come up") (live).
 *   nv_tb_egpu_f40b_fired_teardown() f40b_fires++ ONLY; state untouched.
 *                                   Called by A7's shutdown-path timeout (a
 *                                   teardown artifact, not a usability event)
 *                                   (live).
 *   nv_tb_egpu_recovery_succeeded() recovery_count++; last_recovery_ns := now;
 *                                   state -> healthy. Reserved for A9.
 *   nv_tb_egpu_recovery_failed()    recovery_failures++; state -> lost-permanent
 *                                   if permanent != 0 else lost-temporary.
 *                                   Reserved for A9.
 */
void nv_tb_egpu_f40b_fired(void);
void nv_tb_egpu_f40b_fired_teardown(void);
void nv_tb_egpu_recovery_succeeded(void);
void nv_tb_egpu_recovery_failed(int permanent);

#endif /* _NV_TB_EGPU_METRICS_H_ */
