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
#include <xen/sched.h>
#include <asm/vtimer.h>

#define vtr_to_nr_pre_bits(v)     ((((uint32_t)(v) >> 26) & 7) + 1)
#define vtr_to_nr_apr_regs(v)     (1 << (vtr_to_nr_pre_bits(v) - 5))

#define ESR_ELx_SYS64_ISS_CRM_SHIFT 1
#define ESR_ELx_SYS64_ISS_CRM_MASK (0xf << ESR_ELx_SYS64_ISS_CRM_SHIFT)

#define ICC_IAR1_EL1_SPURIOUS    0x3ff
#define VGIC_MAX_SPI             1019
#define VGIC_MIN_LPI             8192

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

static int vgic_v3_get_group(const union hsr hsr)
{
    uint8_t crm = (hsr.bits & ESR_ELx_SYS64_ISS_CRM_MASK) >>
                   ESR_ELx_SYS64_ISS_CRM_SHIFT;

    return crm != 8;
}

static unsigned int gic_get_num_lrs(void)
{
    uint32_t vtr;

    vtr = READ_SYSREG32(ICH_VTR_EL2);
    return (vtr & GICH_VTR_NRLRGS) + 1;
}

static int vgic_v3_highest_priority_lr(struct cpu_user_regs *regs,
                                       uint32_t vmcr, uint64_t *lr_val)
{
    unsigned int i, lr = -1;
    unsigned int used_lrs =  gic_get_num_lrs();
    uint8_t priority = GICV3_IDLE_PRIORITY;

    for ( i = 0; i < used_lrs; i++ )
    {
        uint64_t val =  gicv3_ich_read_lr(i);
        uint8_t lr_prio = (val & ICH_LR_PRIORITY_MASK) >> ICH_LR_PRIORITY_SHIFT;

        /* Not pending in the state? */
        if ( (val & ICH_LR_STATE) != ICH_LR_PENDING_BIT )
            continue;

        /* Group-0 interrupt, but Group-0 disabled? */
        if ( !(val & ICH_LR_GROUP) && !(vmcr & ICH_VMCR_ENG0_MASK) )
            continue;

        /* Group-1 interrupt, but Group-1 disabled? */
        if ( (val & ICH_LR_GROUP) && !(vmcr & ICH_VMCR_ENG1_MASK) )
            continue;

        /* Not the highest priority? */
        if ( lr_prio >= priority )
            continue;

        /* This is a candidate */
        priority = lr_prio;
        *lr_val = val;
        lr = i;
    }

    if ( lr == -1 )
        *lr_val = ICC_IAR1_EL1_SPURIOUS;

    return lr;
}

static int vgic_v3_get_highest_active_priority(void)
{
    unsigned int i;
    uint32_t hap = 0;
    uint8_t nr_apr_regs = vtr_to_nr_apr_regs(READ_SYSREG32(ICH_VTR_EL2));

    for ( i = 0; i < nr_apr_regs; i++ )
    {
        uint32_t val;

        /*
         * The ICH_AP0Rn_EL2 and ICH_AP1Rn_EL2 registers
         * contain the active priority levels for this VCPU
         * for the maximum number of supported priority
         * levels, and we return the full priority level only
         * if the BPR is programmed to its minimum, otherwise
         * we return a combination of the priority level and
         * subpriority, as determined by the setting of the
         * BPR, but without the full subpriority.
         */
        val  = vgic_v3_read_ap0rn(i);
        val |= vgic_v3_read_ap1rn(i);
        if ( !val )
        {
            hap += 32;
            continue;
        }

        return (hap +  __ffs(val)) << vgic_v3_bpr_min();
    }

    return GICV3_IDLE_PRIORITY;
}

/*
 * Convert a priority to a preemption level, taking the relevant BPR
 * into account by zeroing the sub-priority bits.
 */
static uint8_t vgic_v3_pri_to_pre(uint8_t pri, uint32_t vmcr, int grp)
{
    unsigned int bpr;

    if ( !grp )
        bpr = vgic_v3_get_bpr0(vmcr) + 1;
    else
        bpr = vgic_v3_get_bpr1(vmcr);

    return pri & (GENMASK(7, 0) << bpr);
}

/*
 * The priority value is independent of any of the BPR values, so we
 * normalize it using the minumal BPR value. This guarantees that no
 * matter what the guest does with its BPR, we can always set/get the
 * same value of a priority.
 */
static void vgic_v3_set_active_priority(uint8_t pri, uint32_t vmcr, int grp)
{
    uint8_t pre, ap;
    uint32_t val;
    int apr;

    pre = vgic_v3_pri_to_pre(pri, vmcr, grp);
    ap = pre >> vgic_v3_bpr_min();
    apr = ap / 32;

    if ( !grp )
    {
        val = vgic_v3_read_ap0rn(apr);
        vgic_v3_write_ap0rn(val | BIT(ap % 32), apr);
    }
    else
    {
        val = vgic_v3_read_ap1rn(apr);
        vgic_v3_write_ap1rn(val | BIT(ap % 32), apr);
    }
}

static void vgic_v3_read_iar(struct cpu_user_regs *regs, uint32_t vmcr, int rt)
{
    uint64_t lr_val;
    uint8_t lr_prio, pmr;
    int lr, grp;
    const union hsr hsr = { .bits = regs->hsr };

    grp = vgic_v3_get_group(hsr);

    lr = vgic_v3_highest_priority_lr(regs, vmcr, &lr_val);
    if ( lr < 0 )
        goto spurious;

    if ( grp != !!(lr_val & ICH_LR_GROUP) )
        goto spurious;

    pmr = (vmcr & ICH_VMCR_PMR_MASK) >> ICH_VMCR_PMR_SHIFT;
    lr_prio = (lr_val & ICH_LR_PRIORITY_MASK) >> ICH_LR_PRIORITY_SHIFT;
    if ( pmr <= lr_prio )
        goto spurious;

    if ( vgic_v3_get_highest_active_priority() <=
         vgic_v3_pri_to_pre(lr_prio, vmcr, grp) )
        goto spurious;

    lr_val &= ~ICH_LR_STATE;
    /* No active state for LPIs */
    if ( (lr_val & ICH_LR_VIRTUAL_ID_MASK) <= VGIC_MAX_SPI )
        lr_val |= ICH_LR_ACTIVE_BIT;

    gicv3_ich_write_lr(lr, lr_val);
    vgic_v3_set_active_priority(lr_prio, vmcr, grp);
    set_user_reg(regs, rt,  lr_val & ICH_LR_VIRTUAL_ID_MASK);

    return;

spurious:
     set_user_reg(regs, rt, ICC_IAR1_EL1_SPURIOUS);
}

static int vgic_v3_find_active_lr(int intid, uint64_t *lr_val)
{
    int i;
    unsigned int used_lrs =  gic_get_num_lrs();

    for ( i = 0; i < used_lrs; i++ )
    {
        uint64_t val = gicv3_ich_read_lr(i);

        if ( (val & ICH_LR_VIRTUAL_ID_MASK) == intid &&
            (val & ICH_LR_ACTIVE_BIT) )
        {
            *lr_val = val;
            return i;
        }
    }

    *lr_val = ICC_IAR1_EL1_SPURIOUS;
    return -1;
}

static int vgic_v3_clear_highest_active_priority(void)
{
    int i;
    uint32_t hap = 0;
    uint8_t nr_apr_regs = vtr_to_nr_apr_regs(READ_SYSREG32(ICH_VTR_EL2));

    for ( i = 0; i < nr_apr_regs; i++ )
    {
        uint32_t ap0, ap1;
        int c0, c1;

        ap0 = vgic_v3_read_ap0rn(i);
        ap1 = vgic_v3_read_ap1rn(i);
        if ( !ap0 && !ap1 )
        {
            hap += 32;
            continue;
        }

        c0 = ap0 ? __ffs(ap0) : 32;
        c1 = ap1 ? __ffs(ap1) : 32;

        /* Always clear the LSB, which is the highest priority */
        if ( c0 < c1 )
        {
            ap0 &= ~BIT(c0);
            vgic_v3_write_ap0rn(ap0, i);
            hap += c0;
        }
        else
        {
            ap1 &= ~BIT(c1);
            vgic_v3_write_ap1rn(ap1, i);
            hap += c1;
        }

        /* Rescale to 8 bits of priority */
        return hap << vgic_v3_bpr_min();
    }

    return GICV3_IDLE_PRIORITY;
}

static void vgic_v3_clear_active_lr(int lr, uint64_t lr_val)
{
    lr_val &= ~ICH_LR_ACTIVE_BIT;
    if ( lr_val & ICH_LR_HW )
    {
        uint32_t pid;

        pid = (lr_val & ICH_LR_PHYS_ID_MASK) >> ICH_LR_PHYS_ID_SHIFT;
        WRITE_SYSREG32(pid, ICC_DIR_EL1);
    }
    gicv3_ich_write_lr(lr, lr_val);
}

static void vgic_v3_bump_eoicount(void)
{
    uint32_t hcr;

    hcr = READ_SYSREG32(ICH_HCR_EL2);
    hcr += 1 << ICH_HCR_EOIcount_SHIFT;
    WRITE_SYSREG32(hcr, ICH_HCR_EL2);
}
 
static void vgic_v3_write_eoir(struct cpu_user_regs *regs, uint32_t vmcr,
                               int rt)
{
    uint64_t lr_val;
    uint8_t lr_prio, act_prio;
    int lr, grp;
    const union hsr hsr = { .bits = regs->hsr };
    register_t vid = get_user_reg(regs, hsr.sysreg.reg);

    grp = vgic_v3_get_group(hsr);

    /* Drop priority in any case */
    act_prio = vgic_v3_clear_highest_active_priority();

    /* If EOIing an LPI, no deactivate to be performed */
    if ( vid >= VGIC_MIN_LPI )
        return;

    /* EOImode == 1, nothing to be done here */
    if ( vmcr & ICH_VMCR_EOIM_MASK )
        return;

    lr = vgic_v3_find_active_lr(vid, &lr_val);
    if ( lr == -1 )
    {
        vgic_v3_bump_eoicount();
        return;
    }

    lr_prio = (lr_val & ICH_LR_PRIORITY_MASK) >> ICH_LR_PRIORITY_SHIFT;

    /* If priorities or group do not match, the guest has fscked-up. */
    if ( grp != !!(lr_val & ICH_LR_GROUP) ||
         vgic_v3_pri_to_pre(lr_prio, vmcr, grp) != act_prio )
        return;

    /* Let's now perform the deactivation */
    vgic_v3_clear_active_lr(lr, lr_val);
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

    case HSR_SYSREG_ICC_IAR1_EL1:
        fn = vgic_v3_read_iar;
        break;

    case HSR_SYSREG_ICC_EOIR1_EL1:
        fn = vgic_v3_write_eoir;
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
