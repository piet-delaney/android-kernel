/*
 * arch/arm/mach-s5pv210/cpuidle.c
 *
 * Copyright (c) Samsung Electronics Co. Ltd
 * Copyright (c) 2012 - Will Tisdale <willtisdale@gmail.com>
 *
 * CPU idle driver for S5PV210
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

#ifdef CONFIG_S5P_IDLE2
#include <mach/idle2.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif /* CONFIG_DEBUG_FS */

#define STATE_C1	1
#define STATE_C2	2
#define STATE_C3	3
static unsigned int bail_C2, bail_C3;
static unsigned int request_C2, request_C3, enter_C2, enter_C3, done_C2, done_C3, done_C1;
static unsigned long long time_in_state[3];
static unsigned char idle_state;
static unsigned long idle_time;
#define IDLE2_VERSION 	350
#endif /* CONFIG_S5P_IDLE2 */

inline static void s5p_enter_idle(void)
{
	unsigned long tmp;

	tmp = __raw_readl(S5P_IDLE_CFG);
	tmp &= ~((3<<30)|(3<<28)|(1<<0));
	tmp |= ((2<<30)|(2<<28));
	__raw_writel(tmp, S5P_IDLE_CFG);

	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	__raw_writel(tmp, S5P_PWR_CFG);

	cpu_do_idle();
}

/* Actual code that puts the SoC in different idle states */
inline static int s5p_enter_idle_normal(struct cpuidle_device *device,
				struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;

	local_irq_disable();
	do_gettimeofday(&before);

	s5p_enter_idle();

	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
			(after.tv_usec - before.tv_usec);
	return idle_time;
}

static DEFINE_PER_CPU(struct cpuidle_device, s5p_cpuidle_device);

static struct cpuidle_driver s5p_idle_driver = {
	.name =         "s5p_idle",
	.owner =        THIS_MODULE,
};

#ifdef CONFIG_S5P_IDLE2
inline static unsigned long long report_state_time(unsigned char idle_state)
{
	if (unlikely(!time_in_state[idle_state]))
		return 0;
	else
		return time_in_state[idle_state];
}

inline static unsigned int report_average_residency(unsigned char idle_state)
{
	unsigned long long time = 0;
	time = report_state_time(idle_state);
	if (unlikely(!time))
		return 0;

	switch (idle_state) {
	case 0:
		if (likely(done_C1 > 0)) {
			do_div(time, done_C1);
			break;
	}
	case 1:
		if (likely(done_C2 > 0)) {
			do_div(time, done_C2);
			break;
	}
	case 2:
		if (likely(done_C3 > 0)) {
			do_div(time, done_C3);
			break;
	}
	default:
		break;
	}
	return time;
}

inline static int s5p_enter_idle_deep(struct cpuidle_device *device,
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
		WARN_ON(0);
		/* Fallthrough */
		break;
	}

	/*
	 * If we fall through enter C1.
	 * WARNING: WE *must* restore VIC interupts to enter C1.
	 * Failure to do so results in a WDT reboot with no panic
	 * or stacktrace.
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

restore_interrupts:
	do_gettimeofday(&after);

	local_fiq_enable();
	local_irq_enable();

	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
			(after.tv_usec - before.tv_usec);
	time_in_state[idle_state] += (unsigned long long)idle_time;
	idle_state = 0;
	return idle_time;
}

void earlysuspend_active_fn(bool flag)
{
	if (flag)
		idle2_flags |= EARLYSUSPEND_ACTIVE;
	else
		idle2_flags &= ~EARLYSUSPEND_ACTIVE;
	pr_info("idle2: earlysuspend_active: %d\n", flag);
}

static void external_active_fn(bool flag)
{
	if (flag)
		idle2_flags |= EXTERNAL_ACTIVE;
	else
		idle2_flags &= ~EXTERNAL_ACTIVE;
	pr_info("idle2: external_active: %d\n", flag);
}

static void needs_topon_fn(bool flag)
{
	if (flag) {
		/*
		 * If a second request comes in
		 * deal with it by setting a flag.
		 */
		if (idle2_flags & NEEDS_TOPON) {
			idle2_flags |= TOPON_CONC_REQ;
			pr_info("idle2: |= TOPON_CONC_REQ\n");
		} else {
			idle2_flags |= NEEDS_TOPON;
			pr_info("idle2: |= NEEDS_TOPON\n");
		}
	} else {
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
}

static void idle2_external_active_work_fn(struct work_struct *work)
{
	cancel_delayed_work_sync(&idle2_external_inactive_work);
	external_active_fn(true);
}

static void idle2_external_inactive_work_fn(struct work_struct *work)
{
	external_active_fn(false);
}

static void idle2_enable_topon_work_fn(struct work_struct *work)
{
	needs_topon_fn(true);
}

static void idle2_cancel_topon_work_fn(struct work_struct *work)
{
	needs_topon_fn(false);
}

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

void idle2_needs_topon(void)
{
	if (idle2_flags & WORK_INITIALISED)
		queue_work(idle2_wq, &idle2_enable_topon_work);
}

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


void idle2_cancel_topon(unsigned long delay)
{
	if (idle2_flags & WORK_INITIALISED) {
		idle2_flags |= TOPON_CANCEL_PENDING;
		queue_delayed_work(idle2_wq, &idle2_cancel_topon_work, delay);
	}
}

inline static int s5p_idle_prepare(struct cpuidle_device *device)
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

#endif /* CONFIG_S5P_IDLE2 */

static void idle2_enable_hlt(struct work_struct *work)
{
        enable_hlt();
}
static DECLARE_DELAYED_WORK(idle2_enable_hlt_work, idle2_enable_hlt);

/* Initialize CPU idle by registering the idle states */
static int s5p_init_cpuidle(void)
{
	struct cpuidle_device *device;

	disable_hlt();
        schedule_delayed_work(&idle2_enable_hlt_work, 30 * HZ);

#ifdef CONFIG_S5P_IDLE2
	idle2_wq = create_singlethread_workqueue("idle2_workqueue");
	BUG_ON(!idle2_wq);
	INIT_WORK(&idle2_external_active_work, idle2_external_active_work_fn);
	INIT_DELAYED_WORK(&idle2_external_inactive_work, idle2_external_inactive_work_fn);
	INIT_WORK(&idle2_enable_topon_work, idle2_enable_topon_work_fn);
	INIT_DELAYED_WORK(&idle2_cancel_topon_work, idle2_cancel_topon_work_fn);
	idle2_flags |= WORK_INITIALISED;
#endif /* CONFIG_S5P_IDLE2 */

	cpuidle_register_driver(&s5p_idle_driver);

	device = &per_cpu(s5p_cpuidle_device, smp_processor_id());
	device->state_count = 0;

	/* Wait for interrupt state */
	device->states[0].enter = s5p_enter_idle_normal;
	device->states[0].exit_latency = 20;	/* uS */
	device->states[0].target_residency = 20;
	device->states[0].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(device->states[0].name, "C1");
	strcpy(device->states[0].desc, "ARM clock gating - WFI");
	device->state_count++;
	
#ifdef CONFIG_S5P_IDLE2
	/* Deep-Idle top ON Wait for interrupt state */
	device->states[1].enter = s5p_enter_idle_deep;
	device->states[1].exit_latency = 1250;
	device->states[1].target_residency = 6000;
	device->states[1].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(device->states[1].name, "C2");
	strcpy(device->states[1].desc, "ARM Power gating - WFI");
	device->state_count++;

	/* Deep-Idle top OFF Wait for interrupt state */
	device->states[2].enter = s5p_enter_idle_deep;
	device->states[2].exit_latency = 2000;
	device->states[2].target_residency = 6000;
	device->states[2].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(device->states[2].name, "C3");
	strcpy(device->states[2].desc, "ARM/TOP/SUB Power gating - WFI");
	device->state_count++;
	/*
	 * Device prepare isn't required when we are building with
	 * CONFIG_S5P_IDLE2 disabled as there is only one active state
	 * so there is nothing to prepare.
	 */
	device->prepare = s5p_idle_prepare;
#endif

	if (cpuidle_register_device(device)) {
		pr_err("s5p_init_cpuidle: Failed registering\n");
		BUG();
		return -EIO;
	}
#ifdef CONFIG_S5P_IDLE2
	printk(KERN_INFO "cpuidle: IDLE2 support enabled - version 0.%d by <willtisdale@gmail.com>\n", IDLE2_VERSION);

	register_pm_notifier(&idle2_pm_notifier);

	return s5p_init_remap();
#else /* CONFIG_S5P_IDLE2 */
	return 0;
#endif
}

device_initcall(s5p_init_cpuidle);

#if defined(CONFIG_DEBUG_FS) && defined(CONFIG_PM_SLEEP) && defined(CONFIG_S5P_IDLE2)
inline static int idle2_debug_show(struct seq_file *s, void *data)
{
	seq_printf(s, "\n");
	seq_printf(s, "---------------IDLE2 v0.%d State Statistics--------------\n",
		IDLE2_VERSION);
	seq_printf(s, "\n");
	seq_printf(s, "                               C2 (TOP ON)    C3 (TOP OFF)\n");
	seq_printf(s, "----------------------------------------------------------\n");
	seq_printf(s, "CPUidle requested state:      %12u    %12u\n", request_C2, request_C3);
	seq_printf(s, "State entered:                %12u    %12u\n", enter_C2, enter_C3);
	seq_printf(s, "State completed:              %12u    %12u\n", done_C2, done_C3);
	seq_printf(s, "State not completed:          %12u    %12u\n", bail_C2, bail_C3);
	seq_printf(s, "\n");
	seq_printf(s, "Requested & completed:       %12u%%    %11u%%\n",
		done_C2 * 100 / (request_C2 ?: 1),
		done_C3 * 100 / (request_C3 ?: 1));
	seq_printf(s, "Entered & completed:         %12u%%    %11u%%\n",
		done_C2 * 100 / (enter_C2 ?: 1),
		done_C3 * 100 / (enter_C3 ?: 1));
	seq_printf(s, "Failed:                      %12u%%    %11u%%\n",
		bail_C2 * 100 / (enter_C2 ?: 1),
		bail_C3 * 100 / (enter_C3 ?: 1));
	seq_printf(s, "\n");
	seq_printf(s, "-----------------IDLE2 Failure Statistics-----------------\n");
	seq_printf(s, "       RTC     MMC     VIC     I2S    NAND    Gate  C3Gate      DMA\n");
	seq_printf(s, "----------------------------------------------------------\n");
	seq_printf(s, "   %7u %7u %7u %7u %7u %7u %7u %7u\n",
		bail_rtc, bail_mmc, bail_vic, bail_i2s, bail_nand, bail_gating, bail_C3_gating, bail_DMA);
	seq_printf(s, "\n");
	seq_printf(s, "---------------------C1 Fallback Usage--------------------\n");
	seq_printf(s, "State Entered:                %12u\n", done_C1);
	seq_printf(s, "\n");
	seq_printf(s, "Fallback to C1 (Total):      %12u%%\n",
		done_C1 * 100 / ((request_C2 ?: 1) + (request_C3 ?: 1)));
	seq_printf(s, "\n");
	seq_printf(s, "----------------IDLE2 Residency Statistics----------------\n");
	seq_printf(s, "             C1 (Fallback)     C2 (TOP ON)    C3 (TOP OFF)\n");
	seq_printf(s, "----------------------------------------------------------\n");
	seq_printf(s, "Total (us):   %12llu    %12llu    %12llu\n",
		report_state_time(0), report_state_time(1), report_state_time(2));
	seq_printf(s, "Average (us): %12u    %12u    %12u\n",
		report_average_residency(0), report_average_residency(1), report_average_residency(2));
	seq_printf(s, "----------------------------------------------------------\n");
	seq_printf(s, "\n");

	return 0;
}

static int idle2_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, idle2_debug_show, inode->i_private);
}

static const struct file_operations idle2_debug_ops = {
	.open		= idle2_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init idle2_debug_init(void)
{
	struct dentry *dir;
	struct dentry *d;

	dir = debugfs_create_dir("idle2", NULL);
	if (!dir)
		return -ENOMEM;

	d = debugfs_create_file("error_stats", S_IRUGO, dir, NULL,
		&idle2_debug_ops);
	if (!d)
		return -ENOMEM;

	return 0;
}

late_initcall(idle2_debug_init);
#endif