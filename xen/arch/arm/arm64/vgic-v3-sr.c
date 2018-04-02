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

static uint64_t gicv3_ich_read_lr(int lr)
{
    switch ( lr )
    {
    case 0: return READ_SYSREG(ICH_LR0_EL2);
    case 1: return READ_SYSREG(ICH_LR1_EL2);
    case 2: return READ_SYSREG(ICH_LR2_EL2);
    case 3: return READ_SYSREG(ICH_LR3_EL2);
    case 4: return READ_SYSREG(ICH_LR4_EL2);
    case 5: return READ_SYSREG(ICH_LR5_EL2);
    case 6: return READ_SYSREG(ICH_LR6_EL2);
    case 7: return READ_SYSREG(ICH_LR7_EL2);
    case 8: return READ_SYSREG(ICH_LR8_EL2);
    case 9: return READ_SYSREG(ICH_LR9_EL2);
    case 10: return READ_SYSREG(ICH_LR10_EL2);
    case 11: return READ_SYSREG(ICH_LR11_EL2);
    case 12: return READ_SYSREG(ICH_LR12_EL2);
    case 13: return READ_SYSREG(ICH_LR13_EL2);
    case 14: return READ_SYSREG(ICH_LR14_EL2);
    case 15: return READ_SYSREG(ICH_LR15_EL2);
    default:
        unreachable();
    }
}

static void gicv3_ich_write_lr(int lr, uint64_t val)
{
    switch ( lr )
    {
    case 0:
        WRITE_SYSREG(val, ICH_LR0_EL2);
        break;
    case 1:
        WRITE_SYSREG(val, ICH_LR1_EL2);
        break;
    case 2:
        WRITE_SYSREG(val, ICH_LR2_EL2);
        break;
    case 3:
        WRITE_SYSREG(val, ICH_LR3_EL2);
        break;
    case 4:
        WRITE_SYSREG(val, ICH_LR4_EL2);
        break;
    case 5:
        WRITE_SYSREG(val, ICH_LR5_EL2);
        break;
    case 6:
        WRITE_SYSREG(val, ICH_LR6_EL2);
        break;
    case 7:
        WRITE_SYSREG(val, ICH_LR7_EL2);
        break;
    case 8:
        WRITE_SYSREG(val, ICH_LR8_EL2);
        break;
    case 9:
        WRITE_SYSREG(val, ICH_LR9_EL2);
        break;
    case 10:
        WRITE_SYSREG(val, ICH_LR10_EL2);
        break;
    case 11:
        WRITE_SYSREG(val, ICH_LR11_EL2);
        break;
    case 12:
        WRITE_SYSREG(val, ICH_LR12_EL2);
        break;
    case 13:
        WRITE_SYSREG(val, ICH_LR13_EL2);
        break;
    case 14:
        WRITE_SYSREG(val, ICH_LR14_EL2);
        break;
    case 15:
        WRITE_SYSREG(val, ICH_LR15_EL2);
        break;
    default:
        return;
    }
    isb();
}

static void vgic_v3_write_ap0rn(uint32_t val, int n)
{
    switch (n)
    {
    case 0:
        WRITE_SYSREG32(val, ICH_AP0R0_EL2);
        break;
    case 1:
        WRITE_SYSREG32(val, ICH_AP0R1_EL2);
        break;
    case 2:
        WRITE_SYSREG32(val, ICH_AP0R2_EL2);
        break;
    case 3:
        WRITE_SYSREG32(val, ICH_AP0R3_EL2);
        break;
    default:
        unreachable();
    }
}

static void vgic_v3_write_ap1rn(uint32_t val, int n)
{
    switch (n)
    {
    case 0:
        WRITE_SYSREG32(val, ICH_AP1R0_EL2);
        break;
    case 1:
        WRITE_SYSREG32(val, ICH_AP1R1_EL2);
        break;
    case 2:
        WRITE_SYSREG32(val, ICH_AP1R2_EL2);
        break;
    case 3:
        WRITE_SYSREG32(val, ICH_AP1R3_EL2);
        break;
    default:
        unreachable();
    }
}

static uint32_t vgic_v3_read_ap0rn(int n)
{
    uint32_t val;

    switch (n)
    {
    case 0:
        val = READ_SYSREG32(ICH_AP0R0_EL2);
        break;
    case 1:
        val = READ_SYSREG32(ICH_AP0R1_EL2);
        break;
    case 2:
        val = READ_SYSREG32(ICH_AP0R2_EL2);
        break;
    case 3:
        val = READ_SYSREG32(ICH_AP0R3_EL2);
        break;
    default:
        unreachable();
    }

    return val;
}

static uint32_t vgic_v3_read_ap1rn(int n)
{
    uint32_t val;

    switch (n)
    {
    case 0:
        val = READ_SYSREG32(ICH_AP1R0_EL2);
        break;
    case 1:
        val = READ_SYSREG32(ICH_AP1R1_EL2);
        break;
    case 2:
        val = READ_SYSREG32(ICH_AP1R2_EL2);
        break;
    case 3:
        val = READ_SYSREG32(ICH_AP1R3_EL2);
        break;
    default:
        unreachable();
    }

    return val;
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

static void vgic_v3_read_igrpen1(struct cpu_user_regs *regs, uint32_t vmcr,
                                 int rt)
{
    set_user_reg(regs, rt, !!(vmcr & ICH_VMCR_ENG1_MASK));
}

static void vgic_v3_write_igrpen1(struct cpu_user_regs *regs, uint32_t vmcr,
                                  int rt)
{
    register_t val = get_user_reg(regs, rt);

    if ( val & 1 )
        vmcr |= ICH_VMCR_ENG1_MASK;
    else
        vmcr &= ~ICH_VMCR_ENG1_MASK;

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

    case HSR_SYSREG_ICC_IGRPEN1_EL1:
        if ( is_read )
            fn = vgic_v3_read_igrpen1;
        else
            fn = vgic_v3_write_igrpen1;
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
