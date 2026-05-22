/*
 * SPDX-FileCopyrightText: Copyright (c) 1999-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#define  __NO_VERSION__

#include "os-interface.h"
#include "nv-linux.h"

void* NV_API_CALL os_pci_init_handle(
    NvU32 domain,
    NvU8  bus,
    NvU8  slot,
    NvU8  function,
    NvU16 *vendor,
    NvU16 *device
)
{
    struct pci_dev *dev;
    unsigned int devfn = PCI_DEVFN(slot, function);

    if (!NV_MAY_SLEEP())
        return NULL;

    dev = NV_GET_DOMAIN_BUS_AND_SLOT(domain, bus, devfn);
    if (dev != NULL)
    {
        if (vendor) *vendor = dev->vendor;
        if (device) *device = dev->device;
        pci_dev_put(dev); /* TODO: Fix me! (hotplug) */
    }
    return (void *) dev;
}

NV_STATUS NV_API_CALL os_pci_read_byte(
    void *handle,
    NvU32 offset,
    NvU8 *pReturnValue
)
{
    if (offset >= NV_PCIE_CFG_MAX_OFFSET)
    {
        *pReturnValue = 0xff;
        return NV_ERR_NOT_SUPPORTED;
    }
    pci_read_config_byte( (struct pci_dev *) handle, offset, pReturnValue);
    return NV_OK;
}

NV_STATUS NV_API_CALL os_pci_read_word(
    void *handle,
    NvU32 offset,
    NvU16 *pReturnValue
)
{
    if (offset >= NV_PCIE_CFG_MAX_OFFSET)
    {
        *pReturnValue = 0xffff;
        return NV_ERR_NOT_SUPPORTED;
    }
    pci_read_config_word( (struct pci_dev *) handle, offset, pReturnValue);
    return NV_OK;
}

NV_STATUS NV_API_CALL os_pci_read_dword(
    void *handle,
    NvU32 offset,
    NvU32 *pReturnValue
)
{
    if (offset >= NV_PCIE_CFG_MAX_OFFSET)
    {
        *pReturnValue = 0xffffffff;
        return NV_ERR_NOT_SUPPORTED;
    }
    pci_read_config_dword( (struct pci_dev *) handle, offset, pReturnValue);
    return NV_OK;
}

NV_STATUS NV_API_CALL os_pci_write_byte(
    void *handle,
    NvU32 offset,
    NvU8 value
)
{
    if (offset >= NV_PCIE_CFG_MAX_OFFSET)
        return NV_ERR_NOT_SUPPORTED;

    pci_write_config_byte( (struct pci_dev *) handle, offset, value);
    return NV_OK;
}

NV_STATUS NV_API_CALL os_pci_write_word(
    void *handle,
    NvU32 offset,
    NvU16 value
)
{
    if (offset >= NV_PCIE_CFG_MAX_OFFSET)
        return NV_ERR_NOT_SUPPORTED;

    pci_write_config_word( (struct pci_dev *) handle, offset, value);
    return NV_OK;
}

NV_STATUS NV_API_CALL os_pci_write_dword(
    void *handle,
    NvU32 offset,
    NvU32 value
)
{
    if (offset >= NV_PCIE_CFG_MAX_OFFSET)
        return NV_ERR_NOT_SUPPORTED;

    pci_write_config_dword( (struct pci_dev *) handle, offset, value);
    return NV_OK;
}

NvBool NV_API_CALL os_pci_remove_supported(void)
{
    return NV_TRUE;
}

void NV_API_CALL os_pci_remove(
    void *handle
)
{
    pci_stop_and_remove_bus_device(handle);
}

//
// Report whether the GPU is reached over an external, hot-pluggable PCIe
// transport (Thunderbolt or USB4).
//
// Replaces matching Thunderbolt-3-era bridge vendor/device IDs with the
// Linux PCI subsystem's own transport classification, so that GPUs
// tunnelled over TB4 / USB4 (Intel Barlow Ridge, AMD USB4, ...) are
// recognised. Two independent kernel signals are consulted:
//
//   * pci_is_thunderbolt_attached() - true when the device, or a bridge
//     above it, carries the Intel Thunderbolt VSEC. USB4 host routers
//     carry it too, so this covers USB4 as well as classic Thunderbolt.
//   * pci_dev::untrusted - set by the kernel on devices below a
//     firmware-marked external-facing port; the endpoint-local form of
//     the external_facing marker, covering external transports that do
//     not expose the Intel VSEC.
//
// 'handle' is the GPU's struct pci_dev (nv_state_t::handle).
//
NvBool NV_API_CALL os_pci_is_thunderbolt_attached(
    void *handle
)
{
    struct pci_dev *pdev = (struct pci_dev *) handle;
    NvBool tb_attached;
    NvBool untrusted;

    if (pdev == NULL)
        return NV_FALSE;

    tb_attached = pci_is_thunderbolt_attached(pdev) ? NV_TRUE : NV_FALSE;
    untrusted   = pdev->untrusted ? NV_TRUE : NV_FALSE;

    if (!tb_attached && !untrusted)
        return NV_FALSE;

    //
    // Nominal probe-time telemetry: a single line on the rare event that
    // an external GPU is detected. pci_info() is used because both the
    // signals and the struct pci_dev are only available in this layer.
    //
    pci_info(pdev,
        "external GPU detected (thunderbolt-attached=%s, external/untrusted=%s)\n",
        tb_attached ? "yes" : "no",
        untrusted ? "yes" : "no");

    return NV_TRUE;
}

NV_STATUS NV_API_CALL
os_enable_pci_req_atomics(
    void *handle,
    enum os_pci_req_atomics_type type
)
{
#ifdef NV_PCI_ENABLE_ATOMIC_OPS_TO_ROOT_PRESENT
    int ret;
    u16 val;

    switch (type)
    {
        case OS_INTF_PCIE_REQ_ATOMICS_32BIT:
            ret = pci_enable_atomic_ops_to_root(handle,
                                                PCI_EXP_DEVCAP2_ATOMIC_COMP32);
            break;
        case OS_INTF_PCIE_REQ_ATOMICS_64BIT:
            ret = pci_enable_atomic_ops_to_root(handle,
                                                PCI_EXP_DEVCAP2_ATOMIC_COMP64);
            break;
        case OS_INTF_PCIE_REQ_ATOMICS_128BIT:
            ret = pci_enable_atomic_ops_to_root(handle,
                                                PCI_EXP_DEVCAP2_ATOMIC_COMP128);
            break;
        default:
            ret = -1;
            break;
    }

    if (ret == 0)
    {
        /*
         * GPUs that don't support Requester Atomics have its
         * PCI_EXP_DEVCTL2_ATOMIC_REQ always set to 0 even after SW enables it.
         */
        if ((pcie_capability_read_word(handle, PCI_EXP_DEVCTL2, &val) == 0) &&
            (val & PCI_EXP_DEVCTL2_ATOMIC_REQ))
        {
            return NV_OK;
        }
    }
#endif
    return NV_ERR_NOT_SUPPORTED;
}

void NV_API_CALL os_pci_trigger_flr(void *handle)
{
    struct pci_dev *pdev = (struct pci_dev *) handle;
    int ret;

    nv_printf(NV_DBG_INFO,
        "NVRM: %s() Start to trigger FLR\n", __FUNCTION__);
    ret = pci_save_state(pdev);
    if (ret)
    {
        nv_printf(NV_DBG_ERRORS,
            "NVRM: %s() PCI save state failed, Skip FLR\n", __FUNCTION__);
        return;
    }
#if defined(NV_PCIE_RESET_FLR_PRESENT)
    // If PCI_RESET_DO_RESET is not defined in a particular kernel version
    // define it as 0. Boolean value 0 will trigger a reset of the device.
#ifndef PCI_RESET_DO_RESET
#define PCI_RESET_DO_RESET 0
#endif
    ret = pcie_reset_flr(pdev, PCI_RESET_DO_RESET);
    if (ret)
    {
        nv_printf(NV_DBG_ERRORS,
            "NVRM: %s() PCI FLR might have failed\n", __FUNCTION__);
    }
#else
    nv_printf(NV_DBG_ERRORS,
        "NVRM: %s() PCI FLR not supported\n", __FUNCTION__);
#endif
    pci_restore_state(pdev);
    return;
}
