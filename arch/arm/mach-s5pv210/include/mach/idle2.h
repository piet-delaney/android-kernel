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
bool top_enabled __read_mostly;
/* For stat collection */
static u32 bail_vic, bail_mmc, bail_clock;

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

inline static bool idle2_pre_entry_check(void)
{
	unsigned int iter, reg1, reg2, val;
	void __iomem *base_addr;

	if (unlikely(__raw_readl(VA_VIC2 + VIC_RAW_STATUS) & vic_regs[2])) {
		bail_vic++;
		return true;
	}

	/*
	 * Check for HSMMC activity
	 * Less likely than pending VIC interrupts
	 */
	for (iter = 0; iter < 2; iter++) {
		if (unlikely(iter > 1)) {
		pr_err("Invalid ch[%d] for SD/MMC \n", iter);
		return false;
		}
		base_addr = chk_dev_op[iter].base;
		/* Check CMDINHDAT[1] and CMDINHCMD [0] */
		reg1 = readl(base_addr + S3C_HSMMC_PRNSTS);
		/* Check CLKCON [2]: ENSDCLK */
		reg2 = readl(base_addr + S3C_HSMMC_CLKCON);

		if (unlikely((reg1 & (S3C_HSMMC_CMD_INHIBIT | S3C_HSMMC_DATA_INHIBIT))
				|| (reg2 & (S3C_HSMMC_CLOCK_CARD_EN)))) {
			/* Increment bail counter for stats */
			pr_debug("MMC ch[%d] Active \n", iter);
			bail_mmc++;
			return true;
		}
	}

	/*
	 * Check for OneNAND activity
	 * This is rare, so put it last.
	 */
	base_addr = chk_dev_op[3].base;

	val = __raw_readl(base_addr + 0x0000010c);

	if (unlikely(val & 0x1)) {
		pr_debug("%s: check_onenand_op() returns true\n", __func__);
		/* Use the same stat as HSMMC. */
		bail_mmc++;
		return true;
	}

	/* If nothing is true, return false so we can continue */
	return false;
}

/* Only needed when we are entering into C3 */
inline static bool check_clock_gating(void)
{
	unsigned long val;

	/* check clock gating */
	val = __raw_readl(S5P_CLKGATE_IP0);
	if (unlikely(val & (S5P_CLKGATE_IP0_MDMA | S5P_CLKGATE_IP0_PDMA0
					| S5P_CLKGATE_IP0_G3D | S5P_CLKGATE_IP0_PDMA1))) {
		pr_debug("%s: S5P_CLKGATE_IP0 - DMA/3D active\n", __func__);
		/* Increment bail counter for stats */
		bail_clock++;
		return true;
	} else {
		val = __raw_readl(S5P_CLKGATE_IP3);
		if (unlikely(val & (S5P_CLKGATE_IP3_I2C0 | S5P_CLKGATE_IP3_I2C_HDMI_DDC
						| S5P_CLKGATE_IP3_I2C2))) {
			pr_debug("%s: S5P_CLKGATE_IP3 - i2c / HDMI active\n", __func__);
			/* Increment bail counter for stats */
			bail_clock++;
			return true;
		}
	}
	return false;
}

inline static void s5p_clear_vic_interrupts(void)
{
	/*
	 * Save current VIC registers
	 */
	vic_regs[0] = __raw_readl(VA_VIC0 + VIC_INT_ENABLE);
	vic_regs[1] = __raw_readl(VA_VIC1 + VIC_INT_ENABLE);
	vic_regs[2] = __raw_readl(VA_VIC2 + VIC_INT_ENABLE);
	vic_regs[3] = __raw_readl(VA_VIC3 + VIC_INT_ENABLE);

	/*
	 * Clear VIC registers to disable interrupts
	 */
	__raw_writel(0xffffffff, (VA_VIC0 + VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, (VA_VIC1 + VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, (VA_VIC2 + VIC_INT_ENABLE_CLEAR));
	__raw_writel(0xffffffff, (VA_VIC3 + VIC_INT_ENABLE_CLEAR));
}

inline static void s5p_restore_vic_interrupts(void)
{
	/*
	 * Restore and enable saved VIC registers
	 */
	__raw_writel(vic_regs[0], VA_VIC0 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[1], VA_VIC1 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[2], VA_VIC2 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[3], VA_VIC3 + VIC_INT_ENABLE);
}

inline static void idle2_pre_idle_cfg_set(void)
{
	/*
	 * Wakeup Sources: Enable all wakeup sources by unsetting
	 * all bits in S5P_WAKEUP_MASK.
	 * RTC_ALARM, RTC_TICK, TS0 & 1, KEY, MMC0-3, I2S, ST, CEC
	 */
	tmp = __raw_readl(S5P_WAKEUP_MASK);
	tmp &= ~0xFFFFFFFFU;
	__raw_writel(tmp, S5P_WAKEUP_MASK);

	/*
	 * External Interrupts: Save current EINT MASK register
	 * Set all bits to avoid spurious wakeups
	 * Enable EINT 22 & 29 as wakeup sources
	 */
	save_eint_mask = __raw_readl(S5P_EINT_WAKEUP_MASK);
	tmp = save_eint_mask;
	tmp |= 0xFFFFFFFFU;
	tmp &= ~((1 << 22) | (1 << 29));
	__raw_writel(tmp, S5P_EINT_WAKEUP_MASK);

	/*
	 * Set PWR_CFG register to enter IDLE mode
	 * when WFI is called
	 */
	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	tmp |= S5P_CFG_WFI_IDLE;
	__raw_writel(tmp, S5P_PWR_CFG);

	/*
	 * Disable interrupts through SYSCON
	 */
	tmp = __raw_readl(S5P_OTHERS);
	tmp |= S5P_OTHER_SYSC_INTOFF;
	__raw_writel(tmp, S5P_OTHERS);

	/*
	 * Clear wakeup status register
	 */
	__raw_writel(__raw_readl(S5P_WAKEUP_STAT), S5P_WAKEUP_STAT);
}

inline static void idle2_show_wakeup_irq(void)
{
	/*
	 * FIXME: Make this a simple stats export
	 * instead of dmesg spam as wakeups are
	 * handy for debugging performance issues
	 */
	bool flag = true;
	tmp = __raw_readl(S5P_WAKEUP_STAT);
	if (tmp & (1 << 0))
		printk("Woken by EINT\n");
	else if (tmp & (1 << 1))
		printk("Woken by RTC_ALARM\n");
	else if (tmp & (1 << 2))
		printk("Woken by RTC_TICK\n");
	else if (tmp & (1 << 3))
		printk("Woken by TSADC0\n");
	else if (tmp & (1 << 4))
		printk("Woken by TSADC1\n");
	else if (tmp & (1 << 5))
		printk("Woken by KEY\n");
	else if (tmp & (1 << 9))
		printk("Woken by HSMMC0\n");
	else if (tmp & (1 << 10))
		printk("Woken by HSMMC1\n");
	else if (tmp & (1 << 11))
		printk("Woken by HSMMC2\n");
	else if (tmp & (1 << 12))
		printk("Woken by HSMMC3\n");
	else if (tmp & (1 << 13))
		printk("Woken by I2S\n");
	else if (tmp & (1 << 14))
		printk("Woken by ST\n");
	else if (tmp & (1 << 15))
		printk("Woken by HDMI-CEC\n");
	else
		flag = false;

	if (flag)
		printk("*********************\n");

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

inline static int s5p_enter_idle2(bool top_enabled)
{
	if (unlikely(pm_cpu_sleep == NULL)) {
		pr_err("%s: error: no cpu sleep function\n", __func__);
		return -EINVAL;
	}

	/*
	 * Store the resume address in the INFORM0 register
	 */
	__raw_writel(virt_to_phys(s3c_cpu_resume), S5P_INFORM0);

	/*
	 * Check VIC Status again before entering IDLE2 mode.
	 * Return EBUSY if there is an interrupt pending
	 */
	if (unlikely(__raw_readl(VA_VIC2 + VIC_RAW_STATUS) & vic_regs[2]))
		return -EBUSY;

	/* GPIO Power Down Control */
	if (likely(!top_enabled))
		s5p_gpio_pdn_conf();

	/*
	 * Configure IDLE_CFG register
	 * Set ARM_L2_CACHE field to 0b111111
	 *
	 * For TOP block on:
	 * TOP_LOGIC = ON
	 * TOP_MEMORY = ON
	 * ARM_L2_CACHE = Retention
	 * CFG_DIDLE = DEEP
	 *
	 * For TOP block off
	 * TOP_LOGIC = OFF
	 * TOP_MEMORY = OFF
	 * ARM_L2_CACHE = Retention
	 * CFG_DIDLE = DEEP
	 */
	tmp = __raw_readl(S5P_IDLE_CFG);
	tmp &= ~(0x3fU << 26);
	if (unlikely(top_enabled))
		tmp |= (S5P_IDLE_CFG_TL_ON | S5P_IDLE_CFG_TM_ON
			| S5P_IDLE_CFG_L2_RET | S5P_IDLE_CFG_DIDLE);
	else
		tmp |= (S5P_IDLE_CFG_TL_RET | S5P_IDLE_CFG_TM_RET
			| S5P_IDLE_CFG_L2_RET | S5P_IDLE_CFG_DIDLE);
	__raw_writel(tmp, S5P_IDLE_CFG);

	/*
	 * Set configuration for idle entry
	 */
	idle2_pre_idle_cfg_set();

	/*
	 * Enter idle2 mode using the platform suspend code
	 */
	s3c_idle2_cpu_save(0, PLAT_PHYS_OFFSET - PAGE_OFFSET);

	/*
	 * We have resumed from IDLE and returned.
	 * Use platform CPU init code to continue.
	 */
	cpu_init();

	if (likely(!top_enabled)) {
		/*
		 * Release retention of GPIO/MMC/UART IO pads
		 */
		tmp = __raw_readl(S5P_OTHERS);
		tmp |= (S5P_OTHERS_RET_IO | S5P_OTHERS_RET_CF
			| S5P_OTHERS_RET_MMC | S5P_OTHERS_RET_UART);
		__raw_writel(tmp, S5P_OTHERS);
	}

	/*
	 * Restore the saved EINT Wakeup mask
	 */
	__raw_writel(save_eint_mask, S5P_EINT_WAKEUP_MASK);
#ifdef CONFIG_S5P_IDLE2_DEBUG
	idle2_show_wakeup_irq();
#endif
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
			pr_err("%s: failed to get io memory region\n", __func__);
			return -EINVAL;
		}
		/* ioremap for register block */
		if(pdev == &s5p_device_onenand)
			chk_dev_op[i].base = ioremap(res->start+0x00600000, 4096);
		else
			chk_dev_op[i].base = ioremap(res->start, 4096);

		if (!chk_dev_op[i].base) {
			pr_err("%s: failed to remap io region\n", __func__);
			return -EINVAL;
		}
	}
	return 0;
}
