/*
 * Allwinner SoCs display driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _DISP_SYS_INTF_
#define _DISP_SYS_INTF_
#include "asm/cacheflush.h"
#include <linux/pinctrl/consumer.h>
#include <linux/clk-provider.h>

/* cache flush flags */
#define  CACHE_FLUSH_I_CACHE_REGION       0
#define  CACHE_FLUSH_D_CACHE_REGION       1
#define  CACHE_FLUSH_CACHE_REGION         2
#define  CACHE_CLEAN_D_CACHE_REGION       3
#define  CACHE_CLEAN_FLUSH_D_CACHE_REGION 4
#define  CACHE_CLEAN_FLUSH_CACHE_REGION   5

struct disp_gpio_set_t {
	char gpio_name[32];
	int port;
	int port_num;
	int mul_sel;
	int pull;
	int drv_level;
	int data;
	int gpio;
};

#define DISP_IRQ_RETURN IRQ_HANDLED
#define DISP_PIN_STATE_ACTIVE "active"
#define DISP_PIN_STATE_SLEEP "sleep"

void disp_sys_cache_flush(void *address, u32 length, u32 flags);

int disp_sys_register_irq(u32 IrqNo, u32 Flags, void *Handler, void *pArg,
			  u32 DataSize, u32 Prio);
void disp_sys_unregister_irq(u32 IrqNo, void *Handler, void *pArg);
void disp_sys_disable_irq(u32 IrqNo);
void disp_sys_enable_irq(u32 IrqNo);

struct device_node* disp_sys_script_get_root(char *main_name);
/* returns: 0:invalid, 1: int; 2:str, 3: gpio */
int disp_sys_script_get_item(char *main_name, char *sub_name, int value[],
			     int count);

int disp_sys_get_ic_ver(void);

int disp_sys_gpio_request(struct disp_gpio_set_t *gpio_list,
			  u32 group_count_max);
int disp_sys_gpio_request_simple(struct disp_gpio_set_t *gpio_list,
				 u32 group_count_max);
int disp_sys_gpio_release(int p_handler, s32 if_release_to_default_status);

/* direction: 0:input, 1:output */
int disp_sys_gpio_set_direction(u32 p_handler, u32 direction,
				const char *gpio_name);
int disp_sys_gpio_get_value(u32 p_handler, const char *gpio_name);
int disp_sys_gpio_set_value(u32 p_handler, u32 value_to_gpio,
			    const char *gpio_name);
int disp_sys_pin_set_state(char *dev_name, char *name);

int disp_sys_power_enable(char *name);
int disp_sys_power_disable(char *name);
void *disp_sys_malloc(u32 size);

uintptr_t disp_sys_pwm_request(u32 pwm_id);
int disp_sys_pwm_free(uintptr_t p_handler);
int disp_sys_pwm_enable(uintptr_t p_handler);
int disp_sys_pwm_disable(uintptr_t p_handler);
int disp_sys_pwm_config(uintptr_t p_handler, int duty_ns, int period_ns);
int disp_sys_pwm_set_polarity(uintptr_t p_handler, int polarity);

#endif
