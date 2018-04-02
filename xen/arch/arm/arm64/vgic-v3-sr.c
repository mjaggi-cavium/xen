/*
 * xen/arch/arm/arm64/vgic-v3-sr.c
 *
 * Code to emulate group0/group1 traps for handling
 * cavium erratum 30115
 *
 * This file merges code from Linux virt/kvm/arm/hyp/vgic-v3-sr.c
 * which is : Copyright (C) 2012-2015 - ARM Ltd
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * Xen-Merge: Manish Jaggi <manish.jaggi@cavium.com>
 *
 * Ths program is free software; you can redistribute it and/or
 * modify it under the terms and conditions of the GNU General Public
 * License, version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/current.h>
#include <asm/regs.h>
#include <asm/system.h>
#include <asm/traps.h>

/* Provide wrappers to read write VMCR similar to linux */
static uint64_t vgic_v3_read_vmcr(void)
{
    return READ_SYSREG32(ICH_VMCR_EL2);
}

static void vgic_v3_write_vmcr(uint32_t vmcr)
{
    WRITE_SYSREG32(vmcr, ICH_VMCR_EL2);
}

/* vgic_v3_handle_cpuif_access
 * returns: true if the register is emulated
 *          false if not a sysreg
 */
bool vgic_v3_handle_cpuif_access(struct cpu_user_regs *regs)
{
    int rt;
    uint32_t vmcr;
    void (*fn)(struct cpu_user_regs *, u32, int);
    bool is_read;
    uint32_t sysreg;
    bool ret = true;
    const union hsr hsr = { .bits = regs->hsr };

    sysreg = hsr.bits & HSR_SYSREG_REGS_MASK;
    is_read = hsr.sysreg.read;
    /* Disabling interrupts to prevent change in guest state */
    local_irq_disable();
    if ( hsr.ec != HSR_EC_SYSREG )
    {
        ret = false;
        goto end;
    }

    switch ( sysreg )
    {
    default:
        ret = false;
        goto end;
    }
    /* Call the emulation hander */
    vmcr = vgic_v3_read_vmcr();
    rt = hsr.sysreg.reg;
    fn(regs, vmcr, rt);
end:
    local_irq_enable();

    return ret;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
