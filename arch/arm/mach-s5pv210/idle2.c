/*
 * arch/arm/mach-s5pv210/idle2.c
 *
 * Copyright (c) 2012 - Will Tisdale <willtisdale@gmail.com>
 * Portions of code (c) Samsung
 *
 * IDLE2 driver for S5PV210 - Provides additional idle states
 * and associated functions.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <asm/proc-fns.h>
#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>

#include <mach/map.h>
#include <mach/regs-irq.h>
#include <mach/regs-clock.h>
#include <plat/pm.h>
#include <plat/devs.h>
#include <linux/cpuidle.h>

#include <mach/regs-gpio.h>
#include <mach/power-domain.h>
#include <mach/gpio.h>
#include <mach/gpio-herring.h>
#include <linux/cpufreq.h>
#include <linux/interrupt.h>
#include <mach/pm-core.h>
#include <mach/idle2.h>
#include <linux/hrtimer.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>

#define MAX_CHK_DEV		5
#define IDLE2_FREQ		(800 * 1000) /* 800MHz when screen off */
#define DISABLE_FURTHER_CPUFREQ	0x10
#define ENABLE_FURTHER_CPUFREQ	0x20
#define STATE_C1	1
#define STATE_C2	2
#define STATE_C3	3

/* IDLE2 control flags */
static u16 idle2_flags;
#define DISABLED_BY_SUSPEND	(1 << 0)
#define EXTERNAL_ACTIVE		(1 << 1)
#define WORK_INITIALISED	(1 << 2)
#define INACTIVE_PENDING	(1 << 3)
#define EARLYSUSPEND_ACTIVE	(1 << 4)
#define NEEDS_TOPON		(1 << 5)
#define TOPON_CANCEL_PENDING	(1 << 6)
#define TOPON_CONC_REQ		(1 << 7)
#define BLUETOOTH_TIMEOUT	(1 << 8)
#define BLUETOOTH_REQUEST	(1 << 9)
#define UART_TIMEOUT		(1 << 10)
#define UART_REQUEST		(1 << 11)
//#define AUDIO_ACTIVE		(1 << 12)
//#define CPUFREQ_LOCK_ALLOWED	(1 << 13)
//#define CPUFREQ_LOCK_ACTIVE	(1 << 14)

/* For statistics */
/*
 * FIXME: Use array or dump them altogether once it's confirmed working properly.
 * I prefer dumping them, because they probably aren't necessary.
 */
u32 bail_C2, bail_C3;
u32 request_C2, request_C3, enter_C2, enter_C3, done_C2, done_C3, done_C1;
u8 idle_state;
u32 idle_time;
u64 time_in_state[3];
u32 bail_vic, bail_mmc, bail_gating, bail_i2s, bail_rtc, bail_nand, bail_C3_gating, bail_DMA;

/* Work function declarations */
static struct workqueue_struct *idle2_wq;
struct work_struct idle2_external_active_work;
struct delayed_work idle2_external_inactive_work;
struct work_struct idle2_enable_topon_work;
struct delayed_work idle2_cancel_topon_work;
struct delayed_work idle2_lock_cpufreq_work;
struct delayed_work idle2_unlock_cpufreq_work;
bool top_enabled __read_mostly;

/*
 * For saving & restoring state
 */
static unsigned long vic_regs[4];
static unsigned long tmp, val;

#define GPIO_OFFSET		0x20
#define GPIO_CON_PDN_OFFSET	0x10
#define GPIO_PUD_PDN_OFFSET	0x14
#define GPIO_PUD_OFFSET		0x08

/*
 * Specific device list for checking before entering
 * idle2 mode
 */
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

extern void i2sdma_getpos(dma_addr_t *src);
extern unsigned int get_rtc_cnt(void);

inline static bool idle2_pre_entry_check(void)
{
	unsigned int reg1, reg2;
	unsigned char iter;
	dma_addr_t src;
	unsigned int current_cnt;
	void __iomem *base_addr;

	/*
	 * Check for RTC interrupt
	 */
	current_cnt = get_rtc_cnt();
	if (unlikely(current_cnt < 0x40)) {
		bail_rtc++;
		return true;
	}

	/*
	 * Check for HSMMC activity
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
			bail_mmc++;
			return true;
		}
	}

	/*
	 * Check for pending VIC interrupt
	 */
	if (unlikely(__raw_readl(VA_VIC2 + VIC_RAW_STATUS) & vic_regs[2])) {
		bail_vic++;
		return true;
	}

	/*
	 * Check for OneNAND activity
	 */
	base_addr = chk_dev_op[3].base;

	val = __raw_readl(base_addr + 0x0000010c);

	if (unlikely(val & 0x1)) {
		bail_nand++;
		return true;
	}

	/*
	 * Check I2S DMA activity
	 */
	i2sdma_getpos(&src);
	src = src & 0x3FFF;
	src = 0x4000 - src;
	if (unlikely(src < 0x150)) {
		bail_i2s++;
		return true;
	}

	/* check clock gating */
	val = __raw_readl(S5P_CLKGATE_IP0);
	if (unlikely(val & (S5P_CLKGATE_IP0_MDMA | S5P_CLKGATE_IP0_PDMA0
					| S5P_CLKGATE_IP0_PDMA1))) {
		bail_DMA++;
		return true;
	}
	val = __raw_readl(S5P_CLKGATE_IP3);
		if (unlikely(val & (S5P_CLKGATE_IP3_I2C0 | S5P_CLKGATE_IP3_I2C_HDMI_DDC
						| S5P_CLKGATE_IP3_I2C2))) {
		bail_gating++;
		return true;
	}
	val = __raw_readl(S5P_CLKGATE_IP1);
	if (val & S5P_CLKGATE_IP1_USBHOST) {
		bail_gating++;
		return true;
	}

	/* If nothing is true, return false so we can continue */
	return false;
}

inline static bool check_C3_clock_gating(void)
{
	val = __raw_readl(S5P_CLKGATE_IP0);
	if (unlikely(val & S5P_CLKGATE_IP0_G3D)) {
		bail_C3_gating++;
		return true;
	}
	return false;
}

/*
 * Function which actually enters into DEEP-IDLE states
 */
extern void s5p_enter_idle(void);
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
	if (likely(!top_enabled)) {
		void __iomem *gpio_base = S5PV210_GPA0_BASE;
		do {
			/* Keep the previous state in idle2 mode */
			__raw_writel(0xffff, gpio_base + GPIO_CON_PDN_OFFSET);

			/* Pull up-down state in idle2 is same as normal */
			tmp = __raw_readl(gpio_base + GPIO_PUD_OFFSET);
			__raw_writel(tmp, gpio_base + GPIO_PUD_PDN_OFFSET);

			gpio_base += GPIO_OFFSET;
		} while (gpio_base <= S5PV210_MP28_BASE);
	}

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
	 * Setting configuration for idle2 entry...
	 */

	/*
	 * Wakeup Source configuration:
	 * Set all bits, then reset required sources.
	 * Enable RTC_ALARM, RTC_TICK, KEY, I2S, ST
	 */
	tmp |= 0xffffffffU;
	tmp &= ~((1 << 1) | (1 << 2) | (1 << 5)
		| (1 << 13) | (1 << 14));
	__raw_writel(tmp, S5P_WAKEUP_MASK);

	/*
	 * External Interrupt wakeup sources:
	 * Firstly set all bits to avoid spurious wakeups
	 * Enable max8998, modem, NFC, bcm4329, fsa9480, k3g
	 */
	tmp |= 0xffffffffU;
	tmp &= ~((1 << 7) | (1 << 11) | (1 << 12) | (1 << 15)
		| (1 << 20) | (1 << 21) | (1 << 23) | (1 << 29));
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

	/*
	 * Entering idle2 state using platform PM code.
	 */

	/*
	 * Flush all caches before calling
	 * s3c_idle2_cpu_save()
	 */
	flush_cache_all();

	/*
	 * Save CPU state and enter idle2 state
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
	 * EINT Wakeup sources:
	 * Reset all bits as per default
	 */
	tmp &= ~0xffffffffU;
	__raw_writel(tmp, S5P_EINT_WAKEUP_MASK);

	/*
	 * Restore S5P_IDLE_CFG & S5P_PWR_CFG registers
	 */
	tmp = __raw_readl(S5P_IDLE_CFG);
	tmp &= ~(S5P_IDLE_CFG_TL_MASK | S5P_IDLE_CFG_TM_MASK
		| S5P_IDLE_CFG_L2_MASK | S5P_IDLE_CFG_DIDLE);
	tmp |= (S5P_IDLE_CFG_TL_ON | S5P_IDLE_CFG_TM_ON);
	__raw_writel(tmp, S5P_IDLE_CFG);

	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	__raw_writel(tmp, S5P_PWR_CFG);

	return 0;
}

/*
 * Function which controls invocation of DEEP-IDLE states
 * and invokes all the necessary checks and does all the
 * necessary preparations to enter.
 */
inline int s5p_enter_idle_deep(struct cpuidle_device *device,
				struct cpuidle_state *state)
{
	struct timeval before, after;
	char err;
	unsigned char *state_name = (state->name);

	/*
	 * Convert second ascii character of state_name,
	 * (which is the state number) to char.
	 * Looks cludgy, but it works and is cheap.
	 */
	unsigned char requested_state = (state_name[1] - 48);

	do_gettimeofday(&before);

	/*
	 * Check for any soft-expired timers and run them
	 */
	hrtimer_peek_ahead_timers();

	local_irq_disable();
	local_fiq_disable();

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

	switch (requested_state){
	case STATE_C2:
		request_C2++;
		if (unlikely(idle2_pre_entry_check()))
			/* Fallthrough */
			break;
		else
			goto enter_C2_state;
	case STATE_C3:
		request_C3++;
		if (unlikely(idle2_pre_entry_check()))
			/* Fallthrough */
			break;
		else if (unlikely(check_C3_clock_gating())) {
			/*
			 * Tell cpuidle governor that we went into
			 * C2 instead of C3
			 */
			device->last_state = &device->states[1];
			goto enter_C2_state;
		} else
			goto enter_C3_state;
	default:
		/* Fallthrough */
		break;
	}

	/*
	 * If we fall through enter C1.
	 * WARNING: WE *must* restore VIC interupts to enter C1.
	 *
	 * When we return from C1, tell the cpuidle governor we
	 * ended up in C1.
	 * We don't want to be penalised for short residencies
	 * which would harm the chances of C2 or C3 being
	 * selected next time.
	 */
	__raw_writel(vic_regs[0], VA_VIC0 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[1], VA_VIC1 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[2], VA_VIC2 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[3], VA_VIC3 + VIC_INT_ENABLE);
	s5p_enter_idle();
	done_C1++;
	idle_state = 0;
	device->last_state = &device->states[0];
	goto restore_interrupts;

enter_C2_state:
	enter_C2++;
	err = s5p_enter_idle2(true);
	/*
	 * To keep C2 stats accurate and to ensure that the idle
	 * governor doesn't start penalising us for residency times
	 * below target_residency + exit_latency, lie to cpuidle
	 * and tell it we were in C1 if return from C2 with an error.
	 */
	if (unlikely(err)) {
		device->last_state = &device->states[0];
		pr_debug("idle2: C2 exited with error %d\n", err);
		bail_C2++;
	} else {
		done_C2++;
		idle_state = 1;
	}
	goto restore_vic_state;

enter_C3_state:
	enter_C3++;
	err = s5p_enter_idle2(false);
	/*
	 * To keep C3 stats accurate and to ensure that the idle
	 * governor doesn't start penalising us for residency times
	 * below target_residency + exit_latency, lie to cpuidle
	 * and tell it we were in C1 if return from C3 with an error.
	 */
	if (unlikely(err)) {
		device->last_state = &device->states[0];
		pr_debug("idle2: C3 exited with error %d\n", err);
		bail_C3++;
	} else {
		done_C3++;
		idle_state = 2;
	}

restore_vic_state:
	/*
	 * Restore saved VIC registers and enable interrupts
	 */
	__raw_writel(vic_regs[0], VA_VIC0 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[1], VA_VIC1 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[2], VA_VIC2 + VIC_INT_ENABLE);
	__raw_writel(vic_regs[3], VA_VIC3 + VIC_INT_ENABLE);

/*
 * We put this here because we don't want wakeup reasons if
 * we fall back to C1, because it makes the stats meaningless.
 */
#ifdef CONFIG_S5P_IDLE2_STATS
	idle2_update_wakeup_stats();
#endif

restore_interrupts:
	do_gettimeofday(&after);
	local_fiq_enable();
	local_irq_enable();

	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
			(after.tv_usec - before.tv_usec);
	time_in_state[idle_state] += (u64)idle_time;

	idle_state = 0;
	return idle_time;
}

/*
 * Workqueue related functions which control IDLE2 invocation.
 */

static void idle2_lock_cpufreq_work_fn(struct work_struct *work)
{
	cpufreq_driver_target(cpufreq_cpu_get(0), IDLE2_FREQ,
			DISABLE_FURTHER_CPUFREQ);
	pr_info("%s: CPUfreq locked to %dKHz\n", __func__, IDLE2_FREQ);
//	idle2_flags |= CPUFREQ_LOCK_ACTIVE;
}

static void idle2_unlock_cpufreq_work_fn(struct work_struct *work)
{
	cpufreq_driver_target(cpufreq_cpu_get(0), IDLE2_FREQ,
			ENABLE_FURTHER_CPUFREQ);
	pr_info("%s: CPUfreq unlocked from %dKHz\n", __func__, IDLE2_FREQ);
//	idle2_flags &= ~CPUFREQ_LOCK_ACTIVE;
}

// void idle2_cpufreq_lock(bool flag)
// {
// 	if (idle2_flags & EARLYSUSPEND_ACTIVE) {
// 		if (flag && (idle2_flags & AUDIO_ACTIVE))
// 			queue_delayed_work(idle2_wq, &idle2_lock_cpufreq_work, 0);
// 	} else if (!flag && (idle2_flags & CPUFREQ_LOCK_ACTIVE))
// 			queue_delayed_work(idle2_wq, &idle2_unlock_cpufreq_work, 0);
// }

// void idle2_audio_active(bool flag)
// {
// 	if (flag) {
// 		idle2_flags |= AUDIO_ACTIVE;
// 		idle2_cpufreq_lock(true);
// 	} else {
// 		idle2_flags &= ~AUDIO_ACTIVE;
// 		idle2_cpufreq_lock(false);
// 	}
// }

void earlysuspend_active_fn(bool flag)
{
	if (flag) {
		idle2_flags |= EARLYSUSPEND_ACTIVE;
		queue_delayed_work(idle2_wq, &idle2_lock_cpufreq_work, 0);
	} else {
		idle2_flags &= ~EARLYSUSPEND_ACTIVE;
		queue_delayed_work(idle2_wq, &idle2_unlock_cpufreq_work, 0);
	}
	pr_info("idle2: earlysuspend_active: %d\n", flag);
}

static void idle2_external_active_work_fn(struct work_struct *work)
{
	cancel_delayed_work_sync(&idle2_external_inactive_work);
	idle2_flags |= EXTERNAL_ACTIVE;
	pr_info("idle2: |= EXTERNAL_ACTIVE\n");
}

static void idle2_external_inactive_work_fn(struct work_struct *work)
{
	idle2_flags &= ~EXTERNAL_ACTIVE;
	pr_info("idle2: &= ~EXTERNAL_ACTIVE\n");
}

static void idle2_enable_topon_work_fn(struct work_struct *work)
{
	/*
	 * If the flag is already set, it is
	 * a second request, so deal with it
	 * by setting a flag.
	 */
	if (idle2_flags & NEEDS_TOPON) {
		idle2_flags |= TOPON_CONC_REQ;
		pr_info("idle2: |= TOPON_CONC_REQ\n");
	} else {
		idle2_flags |= NEEDS_TOPON;
		pr_info("idle2: |= NEEDS_TOPON\n");
	}
}

static void idle2_cancel_topon_work_fn(struct work_struct *work)
{
	/*
	 * Figure out what the cancel was called by
	 */
	if (idle2_flags & BLUETOOTH_TIMEOUT) {
		idle2_flags &= ~BLUETOOTH_TIMEOUT;
		idle2_flags &= ~BLUETOOTH_REQUEST;
	} else if (idle2_flags & UART_TIMEOUT) {
		idle2_flags &= ~UART_TIMEOUT;
		idle2_flags &= ~UART_REQUEST;
	}
	/*
	 * If a second request is active
	 * clear that flag first.
	 */
	if (idle2_flags & TOPON_CONC_REQ) {
		idle2_flags &= ~TOPON_CONC_REQ;
		pr_info("idle2: &= ~TOPON_CONC_REQ\n");
	} else {
		idle2_flags &= ~NEEDS_TOPON;
		pr_info("idle2: &= ~NEEDS_TOPON\n");
	}
	/*
	 * If no timeout flags are set, clear the cancel
	 * pending flag.
	 */
	if (!((idle2_flags & UART_TIMEOUT)
		&& (idle2_flags & BLUETOOTH_TIMEOUT)))
		idle2_flags &= ~TOPON_CANCEL_PENDING;
}

/*
 * external_active / inactive functions are called for
 * USB vbus activity.
 */
void idle2_external_active(void)
{
	if ((idle2_flags & WORK_INITIALISED)
		&& (!(idle2_flags & EXTERNAL_ACTIVE)
		|| (idle2_flags & INACTIVE_PENDING))) {
		queue_work(idle2_wq, &idle2_external_active_work);
		idle2_flags &= ~INACTIVE_PENDING;
	}
}

void idle2_external_inactive(unsigned long delay)
{
	if ((idle2_flags & WORK_INITIALISED)
		&& (idle2_flags & EXTERNAL_ACTIVE)
		&& (!(idle2_flags & INACTIVE_PENDING))) {
		idle2_flags |= INACTIVE_PENDING;
		queue_delayed_work(idle2_wq, &idle2_external_inactive_work, delay);
	}
}

/*
 * Bluetooth functions are called for bluetooth activity.
 * They also ignore the large number of active calls you can
 * get whilst bluetooth is active.
 */
void idle2_bluetooth_active(void)
{
	if (idle2_flags & WORK_INITIALISED) {
		if (idle2_flags & BLUETOOTH_REQUEST)
			return;
		queue_work(idle2_wq, &idle2_enable_topon_work);
		idle2_flags |= BLUETOOTH_REQUEST;
	}
}

void idle2_bluetooth_timeout(unsigned long delay)
{
	if (idle2_flags & WORK_INITIALISED) {
		idle2_flags |= BLUETOOTH_TIMEOUT;
		queue_delayed_work(idle2_wq, &idle2_cancel_topon_work, delay);
	}
}

/*
 * uart functions are called when the GPS is active
 */
void idle2_uart_active(void)
{
	if (idle2_flags & WORK_INITIALISED) {
		if (idle2_flags & UART_REQUEST)
			return;
		queue_work(idle2_wq, &idle2_enable_topon_work);
		idle2_flags |= UART_REQUEST;
	}
}

void idle2_uart_timeout(unsigned long delay)
{
	if (idle2_flags & WORK_INITIALISED) {
		idle2_flags |= UART_TIMEOUT;
		queue_delayed_work(idle2_wq, &idle2_cancel_topon_work, delay);
	}
}

static int idle2_pm_notify(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	if (event == PM_SUSPEND_PREPARE) {
		idle2_flags |= DISABLED_BY_SUSPEND;
		pr_info("%s: IDLE2 disabled\n", __func__);
	}
	else if (event == PM_POST_SUSPEND) {
		idle2_flags &= ~DISABLED_BY_SUSPEND;
		pr_info("%s: IDLE2 enabled\n", __func__);
	}
	return NOTIFY_OK;
}

static struct notifier_block idle2_pm_notifier = {
	.notifier_call = idle2_pm_notify,
};

/*
 * Functions which are called from cpuidle.c.
 */

inline int s5p_idle_prepare(struct cpuidle_device *device)
{
	if (unlikely(idle2_flags & DISABLED_BY_SUSPEND)) {
		/*
		 * Ignore both DEEP-IDLE states
		 */
		device->states[1].flags |= CPUIDLE_FLAG_IGNORE;
		device->states[2].flags |= CPUIDLE_FLAG_IGNORE;
		return 0;
	}
	if ((idle2_flags & NEEDS_TOPON)
		|| !(idle2_flags & EARLYSUSPEND_ACTIVE)
		|| (idle2_flags & EXTERNAL_ACTIVE)) {
		/*
		 * Ignore DEEP-IDLE TOP block OFF state
		 */
		device->states[1].flags &= ~CPUIDLE_FLAG_IGNORE;
		device->states[2].flags |= CPUIDLE_FLAG_IGNORE;
		return 0;
	} else {
		/*
		 * Enable both DEEP-IDLE states and allow
		 * cpuidle governor to choose the best one
		 */
		device->states[1].flags &= ~CPUIDLE_FLAG_IGNORE;
		device->states[2].flags &= ~CPUIDLE_FLAG_IGNORE;
	}
	return 0;
}

void s5p_init_idle2_work(void)
{
	idle2_wq = create_singlethread_workqueue("idle2_workqueue");
	BUG_ON(!idle2_wq);
	INIT_WORK(&idle2_external_active_work, idle2_external_active_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&idle2_external_inactive_work, idle2_external_inactive_work_fn);
	INIT_WORK(&idle2_enable_topon_work, idle2_enable_topon_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&idle2_cancel_topon_work, idle2_cancel_topon_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&idle2_lock_cpufreq_work, idle2_lock_cpufreq_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&idle2_unlock_cpufreq_work, idle2_unlock_cpufreq_work_fn);
	idle2_flags |= WORK_INITIALISED;
}

int s5p_idle2_post_init(void)
{
	int i = 0;
	struct platform_device *pdev;
	struct resource *res;

	printk(KERN_INFO "cpuidle: IDLE2 support enabled - version 0.%d by <willtisdale@gmail.com>\n", IDLE2_VERSION);

	register_pm_notifier(&idle2_pm_notifier);

	/*
	 * Allocate memory region to access HSMMC & OneNAND registers
	 */
	for (i = 0 ; i < MAX_CHK_DEV ; i++) {

		pdev = chk_dev_op[i].pdev;

		if (pdev == NULL)
			break;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			pr_err("%s: failed to get io memory region\n", __func__);
			return -EINVAL;
		}
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
