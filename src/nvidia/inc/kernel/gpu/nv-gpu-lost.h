/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/*
 * Shared constants and macros for the GPU-lost crash-safety guards.
 *
 * When a GPU falls off the PCIe bus, a set of call sites in the driver
 * would otherwise either stall the GPU lock or trip an assertion while
 * operating on hardware that is no longer there. The guards built from
 * this header bound those paths so a lost GPU is a contained, survivable
 * event rather than a kernel panic or a multi-second lock stall.
 *
 * This header is intentionally self-contained: it only depends on
 * NV_PRINTF / NV_ASSERT being already in scope at the include site, so
 * it can be pulled in from anywhere in the RM tree that already includes
 * the standard utility headers.
 */

#ifndef _NV_GPU_LOST_H_
#define _NV_GPU_LOST_H_

/*
 * Canonical "dead bus" MMIO read return values. A PCIe device that has
 * fallen off the bus completes config/MMIO reads with all-1s after the
 * hardware completion timeout (tens of milliseconds per read).
 */
#define NV_GPU_BUS_DEAD_VALUE_U32       0xFFFFFFFFU
#define NV_GPU_BUS_DEAD_VALUE_U16       0xFFFFU
#define NV_GPU_BUS_DEAD_VALUE_U8        0xFFU

/*
 * Emit an NV_PRINTF exactly once per kernel-module lifetime at the call
 * site. A lost GPU can drive a path repeatedly; this keeps the log to a
 * single line per site so the event is recorded without flooding the
 * kernel log. The latch is function-scope-static, so each call site gets
 * its own independent latch.
 */
#define NV_GPU_LOST_LOG_ONCE(level, fmt, ...)                                  \
    do {                                                                       \
        static int _nv_gpu_lost_logged_ = 0;                                   \
        if (!_nv_gpu_lost_logged_) {                                           \
            _nv_gpu_lost_logged_ = 1;                                          \
            NV_PRINTF(level, fmt, ##__VA_ARGS__);                              \
        }                                                                      \
    } while (0)

/*
 * Cleanup-path assert relaxation. NV_OK and NV_ERR_GPU_IN_FULLCHIP_RESET
 * are the statuses the unguarded assert already accepts; NV_ERR_GPU_IS_LOST
 * is also benign during resource teardown when the GPU has fallen off the
 * bus -- the cleanup is purely host-side bookkeeping at that point, so a
 * lost GPU must not turn teardown into an assertion failure.
 */
#define NV_ASSERT_OR_GPU_LOST(status)                                          \
    NV_ASSERT(((status) == NV_OK) ||                                           \
              ((status) == NV_ERR_GPU_IN_FULLCHIP_RESET) ||                    \
              ((status) == NV_ERR_GPU_IS_LOST))

/*
 * NV_ASSERT_OR_GPU_LOST_OR_RETURN(status) and NV_ASSERT_OR_GPU_LOST_OR_RETURN_VOID(status)
 * mirror NV_ASSERT_OR_RETURN / NV_ASSERT_OR_RETURN_VOID one-for-one, sharing the
 * predicate body above. Applied at cleanup / post-RPC sites that may receive
 * NV_ERR_GPU_IS_LOST from C5-guarded RPC funnels (e.g. _issueRpcAndWait
 * short-circuit returning NV_ERR_GPU_IS_LOST during teardown).
 */
#define NV_ASSERT_OR_GPU_LOST_OR_RETURN(status)                                \
    NV_ASSERT_OR_RETURN(((status) == NV_OK) ||                                 \
                        ((status) == NV_ERR_GPU_IN_FULLCHIP_RESET) ||          \
                        ((status) == NV_ERR_GPU_IS_LOST),                      \
                        (status))

#define NV_ASSERT_OR_GPU_LOST_OR_RETURN_VOID(status)                           \
    NV_ASSERT_OR_RETURN_VOID(((status) == NV_OK) ||                            \
                             ((status) == NV_ERR_GPU_IN_FULLCHIP_RESET) ||     \
                             ((status) == NV_ERR_GPU_IS_LOST))

/*
 * v4 sink-primitive architecture: detector-class enum + cleanupGpuLostStateAtomic.
 *
 * The driver discovers a GPU is off the bus from multiple distinct input
 * classes (post-MMIO-read sentinel, osHandleGpuLost retry-exhausted, GSP
 * heartbeat timeout, AER fatal callback, Q-watchdog DMA wedge,
 * probe-time BAR-allocation failure, kernel-side sysfs disconnect). v4
 * routes every detection input through one idempotent per-GPU primitive
 * so the dual markers (PDB_PROP_GPU_IS_LOST + pci_dev_is_disconnected)
 * are always set together and one canonical log line records which
 * input class fired.
 *
 * Reserved enum slot DETECTOR_UVM_FATAL is consumed by C6 (F1) in
 * Phase 2; the slot is allocated here so Phase 1 vs Phase 2 don't
 * collide on numeric values.
 */
typedef enum {
    DETECTOR_MMIO_DEAD                       = 0,
    DETECTOR_OSHANDLEGPULOST_RETRY_EXHAUSTED = 1,
    DETECTOR_GSP_HEARTBEAT_TIMEOUT           = 2,
    DETECTOR_AER_FATAL                       = 3,
    DETECTOR_QWATCHDOG_DMA_WEDGE             = 4,
    DETECTOR_PROBE_BAR_FAILURE               = 5,
    DETECTOR_SYSFS_DISCONNECTED              = 6,
    DETECTOR_UVM_FATAL                       = 7,   /* reserved for Phase 2 C6 */
} nv_gpu_lost_detector_t;

/*
 * Atomic per-GPU sink-state setter. Idempotent; safe from any detection
 * input. Sets PDB_PROP_GPU_IS_LOST (via gpuSetDisconnectedProperties)
 * AND pci_dev_is_disconnected (via os_pci_set_disconnected). Emits one
 * NV_PRINTF per detector_class per kernel module lifetime, naming which
 * detector input observed the dead bus.
 *
 * Forward declaration of OBJGPU keeps this header self-contained.
 */
struct OBJGPU;
void cleanupGpuLostStateAtomic(struct OBJGPU *pGpu,
                               nv_gpu_lost_detector_t detector_class);

#endif /* _NV_GPU_LOST_H_ */
