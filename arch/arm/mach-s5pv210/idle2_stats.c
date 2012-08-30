/*
 * arch/arm/mach-s5pv210/idle2_stats.c
 *
 * Copyright (c) 2012 - Will Tisdale <willtisdale@gmail.com>
 *
 * IDLE2 statistics for S5PV210
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <mach/idle2.h>
#include <mach/regs-clock.h>
#include <linux/io.h>


extern u32 bail_C2, bail_C3;
extern u32 request_C2, request_C3, enter_C2, enter_C3, done_C2, done_C3, done_C1;
u32 s5p_wakeup_reason[6];
extern u32 bail_vic, bail_mmc, bail_gating, bail_i2s, bail_rtc, bail_nand, bail_C3_gating, bail_DMA;
extern u8 idle_state;
extern u64 time_in_state[3];

void idle2_update_wakeup_stats(void)
{
	unsigned long tmp;
	tmp = __raw_readl(S5P_WAKEUP_STAT);
	if (unlikely (tmp & (1 << 0)))
		s5p_wakeup_reason[0]++;
	if (unlikely (tmp & (1 << 1)))
		s5p_wakeup_reason[1]++;
	if (unlikely (tmp & (1 << 2)))
		s5p_wakeup_reason[2]++;
	if (unlikely (tmp & (1 << 5)))
		s5p_wakeup_reason[3]++;
	if (unlikely (tmp & (1 << 13)))
		s5p_wakeup_reason[4]++;
	if (unlikely (tmp & (1 << 14)))
		s5p_wakeup_reason[5]++;
}

unsigned long long report_state_time(unsigned char idle_state)
{
	if (unlikely(!time_in_state[idle_state]))
		return 0;
	else
		return time_in_state[idle_state];
}

unsigned int report_average_residency(unsigned char idle_state)
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

static int idle2_debug_show(struct seq_file *s, void *data)
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
	seq_printf(s, "      RTC    MMC    VIC    I2S   NAND   Gate C3Gate    DMA\n");
	seq_printf(s, "----------------------------------------------------------\n");
	seq_printf(s, "   %6u %6u %6u %6u %6u %6u %6u %6u\n",
		bail_rtc, bail_mmc, bail_vic, bail_i2s, bail_nand, bail_gating, bail_C3_gating, bail_DMA);
	seq_printf(s, "\n");
	seq_printf(s, "---------------------C1 Fallback Usage--------------------\n");
	seq_printf(s, "State Entered:                %12u\n", done_C1);
	seq_printf(s, "\n");
	seq_printf(s, "Fallback to C1 (Total):      %12u%%\n",
		(done_C1 * 100) / ((request_C2 ?: 1) + request_C3));
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
	seq_printf(s, "-----------------IDLE2 Wakeup Statistics------------------\n");
	seq_printf(s, "                   EINT  ALARM   TICK    KEY    I2S     ST\n");
	seq_printf(s, "----------------------------------------------------------\n");
	seq_printf(s, "                 %6u %6u %6u %6u %6u %6u\n", 
		s5p_wakeup_reason[0], s5p_wakeup_reason[1], s5p_wakeup_reason[2],
		s5p_wakeup_reason[3], s5p_wakeup_reason[4], s5p_wakeup_reason[5]);
	seq_printf(s, "----------------------------------------------------------\n");
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

	d = debugfs_create_file("statistics", S_IRUGO, dir, NULL,
		&idle2_debug_ops);
	if (!d)
		return -ENOMEM;

	return 0;
}

late_initcall(idle2_debug_init);
