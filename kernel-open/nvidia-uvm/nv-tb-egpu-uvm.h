/*
 * SPDX-FileCopyrightText: nvidia-driver-injector contributors
 * SPDX-License-Identifier: MIT
 *
 * nv-tb-egpu-uvm.h — UVM close-path nominal telemetry (addon A4).
 *
 * Public interface for uvm.c.  The helpers below increment/read/decrement
 * a UVM-internal fd refcount and emit a one-line [CLOSE] marker at five
 * UVM lifecycle sites.  On the last-close transition (the open that
 * brings the fd count up from 0 OR the release that brings it back to 0),
 * additionally captures a PMC_BOOT_0 + WPR2 hardware-health snapshot via
 * the nvidia.ko-exported helper tb_egpu_close_diag_pdev.
 *
 * Companion to the RM-side close-path markers in nv-tb-egpu-close.c.
 * The fd-count tracking is UVM-local because UVM has its own
 * /dev/nvidia-uvm lifecycle that does NOT participate in nvl->usage_count.
 *
 * Observability scope: nominal bar only — one line per site, hardware
 * snapshot only on last-close transition, no AER/LnkSta deep-walk.
 *
 * See nv-tb-egpu-uvm.c for the implementation notes.
 */

#ifndef _NV_TB_EGPU_UVM_H_
#define _NV_TB_EGPU_UVM_H_

/*
 * Five helpers — one per UVM lifecycle site.  Each encapsulates its
 * own atomic operation on the internal fd_count, so the uvm.c call
 * sites are pure single-line calls (no inline atomic logic to repeat).
 */
void tb_egpu_uvm_close_diag_at_open(void);                /* uvm-open-entry      */
void tb_egpu_uvm_close_diag_at_release_entry(void);       /* uvm-release-entry   */
void tb_egpu_uvm_close_diag_at_pre_destroy(void);         /* uvm-pre-destroy     */
void tb_egpu_uvm_close_diag_at_post_destroy(void);        /* uvm-post-destroy    */
void tb_egpu_uvm_close_diag_at_release_exit(void);        /* uvm-release-exit    */

#endif /* _NV_TB_EGPU_UVM_H_ */
