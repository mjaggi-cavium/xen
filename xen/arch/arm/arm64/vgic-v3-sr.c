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
#include <asm/gic_v3_defs.h>
#include <asm/regs.h>
#include <asm/system.h>
#include <asm/traps.h>

#define vtr_to_nr_pre_bits(v)     ((((uint32_t)(v) >> 26) & 7) + 1)

/* Provide wrappers to read write VMCR similar to linux */
static uint64_t vgic_v3_read_vmcr(void)
{
    return READ_SYSREG32(ICH_VMCR_EL2);
}

static void vgic_v3_write_vmcr(uint32_t vmcr)
{
    WRITE_SYSREG32(vmcr, ICH_VMCR_EL2);
}

static int vgic_v3_bpr_min(void)
{
    /* See Pseudocode for VPriorityGroup */
    return 8 - vtr_to_nr_pre_bits(READ_SYSREG32(ICH_VTR_EL2));
}

static unsigned int vgic_v3_get_bpr0(uint32_t vmcr)
{
    return (vmcr & ICH_VMCR_BPR0_MASK) >> ICH_VMCR_BPR0_SHIFT;
}

static unsigned int vgic_v3_get_bpr1(uint32_t vmcr)
{
    unsigned int bpr;

    if ( vmcr & ICH_VMCR_CBPR_MASK )
    {
        bpr = vgic_v3_get_bpr0(vmcr);
        if ( bpr < 7 )
            bpr++;
    }
    else
        bpr = (vmcr & ICH_VMCR_BPR1_MASK) >> ICH_VMCR_BPR1_SHIFT;

    return bpr;
}

static void vgic_v3_read_bpr1(struct cpu_user_regs *regs, uint32_t vmcr,
                              int rt)
{
    set_user_reg(regs, rt, vgic_v3_get_bpr1(vmcr));
}

static void vgic_v3_write_bpr1(struct cpu_user_regs *regs, uint32_t vmcr,
                               int rt)
{
    register_t val = get_user_reg(regs, rt);
    uint8_t bpr_min = vgic_v3_bpr_min();

    if ( vmcr & ICH_VMCR_CBPR_MASK )
        return;

    /* Enforce BPR limiting */
    if ( val < bpr_min )
        val = bpr_min;

    val <<= ICH_VMCR_BPR1_SHIFT;
    val &= ICH_VMCR_BPR1_MASK;
    vmcr &= ~ICH_VMCR_BPR1_MASK;
    vmcr |= val;

    vgic_v3_write_vmcr(vmcr);
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

    case HSR_SYSREG_ICC_BPR1_EL1:
        if ( is_read )
            fn = vgic_v3_read_bpr1;
        else
            fn = vgic_v3_write_bpr1;
        break;

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
