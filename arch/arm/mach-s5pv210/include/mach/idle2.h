/* arch/arm/mach-s5pv210/include/mach/idle2.h
 *
 * Copyright (c) Samsung Electronics Co. Ltd
 * Copyright (c) 2012 Will Tisdale - <willtisdale@gmail.com>
 *
 * S5PV210 - IDLE2 functions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


#include <mach/regs-gpio.h>
#include <mach/cpuidle.h>
#include <mach/power-domain.h>
#include <mach/gpio.h>
#include <mach/gpio-herring.h>
#include <linux/cpufreq.h>
#include <linux/interrupt.h>
#include <mach/pm-core.h>

#define MAX_CHK_DEV	5

/* IDLE2 control flags */
static unsigned char idle2_flags;
#define DISABLED_BY_SUSPEND	(1 << 0)
#define EXTERNAL_ACTIVE		(1 << 1)
#define WORK_INITIALISED	(1 << 2)
#define INACTIVE_PENDING	(1 << 3)
#define EARLYSUSPEND_ACTIVE	(1 << 4)
#define NEEDS_TOPON		(1 << 5)
#define TOPON_CANCEL_PENDING	(1 << 6)

static struct workqueue_struct *idle2_wq;
struct work_struct idle2_external_active_work;
struct delayed_work idle2_external_inactive_work;
struct work_struct idle2_enable_topon_work;
struct delayed_work idle2_cancel_topon_work;
bool top_status __read_mostly;

/*
 * For saving & restoring VIC register before entering
 * idle2 mode
 */
static unsigned long vic_regs[4];
static unsigned long tmp;
static unsigned long save_eint_mask;

/* Specific device list for checking before entering
 * idle2 mode
 **/
struct check_device_op {
	void __iomem		*base;
	struct platform_device	*pdev;
};

/* Array of checking devices list */
struct check_device_op chk_dev_op[] = {
#if defined(CONFIG_S3C_DEV_HSMMC)
	{.base = 0, .pdev = &s3c_device_hsmmc0},
#endif
#if defined(CONFIG_S3C_DEV_HSMMC1)
	{.base = 0, .pdev = &s3c_device_hsmmc1},
#endif
#if defined(CONFIG_S3C_DEV_HSMMC2)
	{.base = 0, .pdev = &s3c_device_hsmmc2},
#endif
#if defined(CONFIG_S3C_DEV_HSMMC3)
	{.base = 0, .pdev = &s3c_device_hsmmc3},
#endif
	{.base = 0, .pdev = &s5p_device_onenand},
	{.base = 0, .pdev = NULL},
};

#define S3C_HSMMC_PRNSTS	(0x24)
#define S3C_HSMMC_CLKCON	(0x2c)
#define S3C_HSMMC_CMD_INHIBIT	0x00000001
#define S3C_HSMMC_DATA_INHIBIT	0x00000002
#define S3C_HSMMC_CLOCK_CARD_EN	0x0004


/* If SD/MMC interface is working: return = true */
inline static bool check_sdmmc_op(unsigned int ch)
{
	unsigned int reg1, reg2;
	void __iomem *base_addr;

	if (unlikely(ch > 2)) {
		pr_err("Invalid ch[%d] for SD/MMC \n", ch);
		return false;
	}

	base_addr = chk_dev_op[ch].base;
	/* Check CMDINHDAT[1] and CMDINHCMD [0] */
	reg1 = readl(base_addr + S3C_HSMMC_PRNSTS);
	/* Check CLKCON [2]: ENSDCLK */
	reg2 = readl(base_addr + S3C_HSMMC_CLKCON);

	if (unlikely((reg1 & (S3C_HSMMC_CMD_INHIBIT | S3C_HSMMC_DATA_INHIBIT)) ||
			(reg2 & (S3C_HSMMC_CLOCK_CARD_EN))))
		return true;
	else
		return false;
}

/* Check all sdmmc controller */
inline static bool loop_sdmmc_check(void)
{
	unsigned int iter;

	for (iter = 0; iter < 3; iter++) {
		if (unlikely(check_sdmmc_op(iter))) {
			pr_debug("%s: %d returns true\n", __func__, iter);
			return true;
		}
	}
	return false;
}

/* Check onenand is working or not */

/* ONENAND_IF_STATUS(0xB060010C)
 * ORWB[0] = 	1b : busy
 * 		0b : Not busy
 **/
inline static bool check_onenand_op(void)
{
	unsigned int val;
	void __iomem *base_addr;

	base_addr = chk_dev_op[3].base;

	val = __raw_readl(base_addr + 0x0000010c);

	if (unlikely(val & 0x1)) {
		pr_debug("%s: check_onenand_op() returns true\n", __func__);
		return true;
	}
	return false;
}

inline static bool check_clock_gating(void)
{
	unsigned long val;

	/* check clock gating */
	val = __raw_readl(S5P_CLKGATE_IP0);
	if (unlikely(val & (S5P_CLKGATE_IP0_MDMA | S5P_CLKGATE_IP0_PDMA0
					| S5P_CLKGATE_IP0_G3D | S5P_CLKGATE_IP0_PDMA1))) {
		pr_debug("%s: S5P_CLKGATE_IP0 - DMA/3D active\n", __func__);
		return true;
	}

	val = __raw_readl(S5P_CLKGATE_IP3);
	if (unlikely(val & (S5P_CLKGATE_IP3_I2C0 | S5P_CLKGATE_IP3_I2C_HDMI_DDC
					| S5P_CLKGATE_IP3_I2C2))) {
		pr_debug("%s: S5P_CLKGATE_IP3 - i2c / HDMI active\n", __func__);
		return true;
	}

	return false;
}

inline static bool enter_idle2_check(void)
{
	if (unlikely(loop_sdmmc_check()))
		return true;
	if (unlikely(check_clock_gating() || check_onenand_op()))
		return true;
	else
		return false;

}

inline static bool s5p_vic_interrupt_pending(void)
{
	if (unlikely((__raw_readl(VA_VIC0 + VIC_RAW_STATUS) & vic_regs[0]) |
		(__raw_readl(VA_VIC1 + VIC_RAW_STATUS) & vic_regs[1]) |
		(__raw_readl(VA_VIC2 + VIC_RAW_STATUS) & vic_regs[2]) |
		(__raw_readl(VA_VIC3 + VIC_RAW_STATUS) & vic_regs[3])))
		return true;
	else
		return false;
}

inline static void s5p_clear_vic_interrupts(void)
{
	vic_regs[0] = __raw_readl(VA_VIC0 + VIC_INT_ENABLE);
	vic_regs[1] = __raw_readl(VA_VIC1 + VIC_INT_ENABLE);
	vic_regs[2] = __raw_readl(VA_VIC2 + VIC_INT_ENABLE);
	vic_regs[3] = __raw_readl(VA_VIC3 + VIC_INT_ENABLE);

	__raw_writel(0xffffffff, (VA_VIC0 + VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, (VA_VIC1 + VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, (VA_VIC2 + VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, (VA_VIC3 + VIC_INT_ENABLE_CLEAR));
}

inline static void s5p_restore_vic_interrupts(void)
{
	__raw_writel(vic_regs[0], VA_VIC0 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[1], VA_VIC1 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[2], VA_VIC2 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[3], VA_VIC3 + VIC_INT_ENABLE);
}

inline static void idle2_pre_idle_cfg_set(void)
{
	/* Power mode Config setting */
	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	tmp |= S5P_CFG_WFI_IDLE;
	__raw_writel(tmp, S5P_PWR_CFG);

	/* SYSCON_INT_DISABLE */
	tmp = __raw_readl(S5P_OTHERS);
	tmp |= S5P_OTHER_SYSC_INTOFF;
	__raw_writel(tmp, S5P_OTHERS);

	/* Wakeup source configuration for idle2 */
	/* Wake up from EINT22 and 29 */
	save_eint_mask = __raw_readl(S5P_EINT_WAKEUP_MASK);
	tmp = s3c_irqwake_eintmask;
	tmp &= ~((1 << 22) | (1 << 29));
	__raw_writel(tmp, S5P_EINT_WAKEUP_MASK);

	/* Wake from RTC tick and i2s */
	tmp = s3c_irqwake_intmask;
	tmp &= ~((1 << 2) | (1 << 13));
	__raw_writel(tmp, S5P_WAKEUP_MASK);

	/* Clear wakeup status register */
	__raw_writel(0x0, S5P_WAKEUP_STAT);
}

inline static void idle2_post_wake_cfg_reset(void)
{
	// FIXME: Not entirely sure if this is needed

	/* Reset the IDLE CFG register */
	tmp = __raw_readl(S5P_IDLE_CFG);
	tmp &= ~(S5P_IDLE_CFG_TL_MASK | S5P_IDLE_CFG_TM_MASK |
		S5P_IDLE_CFG_L2_MASK | S5P_IDLE_CFG_DIDLE);
	tmp |= (S5P_IDLE_CFG_TL_ON | S5P_IDLE_CFG_TM_ON);
	__raw_writel(tmp, S5P_IDLE_CFG);

	/* Reset the Power CFG register */
	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	__raw_writel(tmp, S5P_PWR_CFG);

	/* Restore the EINT Wakeup mask */
	__raw_writel(save_eint_mask, S5P_EINT_WAKEUP_MASK);
}

/*
 * Before entering, idle2 mode GPIO Power Down Mode
 * Configuration register has to be set with same state
 * in Normal Mode
 */
#define GPIO_OFFSET		0x20
#define GPIO_CON_PDN_OFFSET	0x10
#define GPIO_PUD_PDN_OFFSET	0x14
#define GPIO_PUD_OFFSET		0x08

inline static void s5p_gpio_pdn_conf(void)
{
	void __iomem *gpio_base = S5PV210_GPA0_BASE;
	unsigned int val;

	do {
		/* Keep the previous state in idle2 mode */
		__raw_writel(0xffff, gpio_base + GPIO_CON_PDN_OFFSET);

		/* Pull up-down state in idle2 is same as normal */
		val = __raw_readl(gpio_base + GPIO_PUD_OFFSET);
		__raw_writel(val, gpio_base + GPIO_PUD_PDN_OFFSET);

		gpio_base += GPIO_OFFSET;

	} while (gpio_base <= S5PV210_MP28_BASE);
}

inline static int s5p_enter_idle2(bool top_status)
{
	if (unlikely(pm_cpu_sleep == NULL)) {
		pr_err("%s: error: no cpu sleep function\n", __func__);
		return -EINVAL;
	}

	/* ensure at least INFORM0 has the resume address */
	__raw_writel(virt_to_phys(s3c_cpu_resume), S5P_INFORM0);

	/* Save and disable VIC interrupts */
	s5p_clear_vic_interrupts();

	/*
	 * Check VIC Status again before entering IDLE2 mode.
	 * Return EBUSY if there is an interrupt pending
	 */
	if (unlikely(s5p_vic_interrupt_pending())) {
		pr_debug("%s: VIC interrupt pending, bailing!\n", __func__);
		s5p_restore_vic_interrupts();
		return -EBUSY;
	}

	/* GPIO Power Down Control */
	if (likely(!top_status))
		s5p_gpio_pdn_conf();

	/* Configure IDLE_CFG register */
	tmp = __raw_readl(S5P_IDLE_CFG);
	/* No idea what this shift is for... */
	tmp &= ~(0x3f << 26);
	if (unlikely(top_status))
		tmp |= (S5P_IDLE_CFG_TL_ON | S5P_IDLE_CFG_TM_ON
			| S5P_IDLE_CFG_L2_RET | S5P_IDLE_CFG_DIDLE);
	else
		tmp |= (S5P_IDLE_CFG_TL_RET | S5P_IDLE_CFG_TM_RET
			| S5P_IDLE_CFG_L2_RET | S5P_IDLE_CFG_DIDLE);
	__raw_writel(tmp, S5P_IDLE_CFG);

	/* Set configuration for idle entry */
	idle2_pre_idle_cfg_set();

	/* Enter idle2 mode */
	s3c_idle2_cpu_save(0, PLAT_PHYS_OFFSET - PAGE_OFFSET);

	/*
	 * We have resumed from IDLE and returned here.
	 * Use platform CPU init code to continue.
	 */
	cpu_init();

	if (likely(!top_status)) {
		/* Release retention of GPIO/MMC/UART IO */
		tmp = __raw_readl(S5P_OTHERS);
		tmp |= (S5P_OTHERS_RET_IO | S5P_OTHERS_RET_CF
			| S5P_OTHERS_RET_MMC | S5P_OTHERS_RET_UART);
		__raw_writel(tmp, S5P_OTHERS);
	}

	/* Post idle configuration restore */
	idle2_post_wake_cfg_reset();
	s5p_restore_vic_interrupts();
	return 0;
}

int s5p_init_remap(void)
{
	int i = 0;
	struct platform_device *pdev;
	struct resource *res;
	/* Allocate memory region to access IP's directly */
	for (i = 0 ; i < MAX_CHK_DEV ; i++) {

		pdev = chk_dev_op[i].pdev;

		if (pdev == NULL)
			break;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			pr_err("failed to get io memory region\n");
			return -EINVAL;
		}
		/* ioremap for register block */
		if(pdev == &s5p_device_onenand)
			chk_dev_op[i].base = ioremap(res->start+0x00600000, 4096);
		else
			chk_dev_op[i].base = ioremap(res->start, 4096);

		if (!chk_dev_op[i].base) {
			pr_err("failed to remap io region\n");
			return -EINVAL;
		}
	}
	return 0;
}
