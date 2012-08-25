/* arch/arm/mach-s5pv210/include/mach/cpuidle.h
 *
 * Copyright (c) 2010 Samsung Electronics - Jaecheol Lee <jc.lee@samsung>
 * Copyright (c) 2012 Will Tisdale - <willtisdale@gmail.com>
 *
 * S5PV210 - CPUIDLE support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifdef CONFIG_S5P_IDLE2
/*
 * For external function calls.
 * In here to avoid importing the entire of idle2.h
 * into external c files.
 */
extern void idle2_external_active(void);
extern void idle2_external_inactive(unsigned long delay);
extern void earlysuspend_active_fn(bool flag);
extern void idle2_cancel_topon(unsigned long delay);
extern void idle2_needs_topon(void);
extern void idle2_bluetooth_active(void);
extern void idle2_bluetooth_timeout(unsigned long delay);
extern void idle2_uart_active(void);
extern void idle2_uart_timeout(unsigned long delay);
#endif