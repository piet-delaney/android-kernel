/* arch/arm/mach-s5pv210/include/mach/idle2.h
 *
 * Copyright (c) Samsung Electronics Co. Ltd
 * Copyright (c) 2012 Will Tisdale - <willtisdale@gmail.com>
 *
 * S5PV210 - IDLE2 external calls
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

extern void idle2_external_active(void);
extern void idle2_external_inactive(unsigned long delay);
extern void earlysuspend_active_fn(bool flag);
extern void idle2_uart_active(bool flag);
extern void idle2_update_wakeup_stats(void);
extern void idle2_kill(bool kill, u16 timeout);
extern void idle2_bluetooth_irq_active(bool kill, u16 timeout);

#define IDLE2_VERSION 	391
