/*
 * drivers/power/axp/axp-regulator.h
 * (C) Copyright 2010-2016
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Pannan <pannan@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#ifndef AXP_REGULATOR_H
#define AXP_REGULATOR_H

#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

struct  axp_reg_init {
	struct regulator_init_data axp_reg_init_data;
	struct axp_regulator_info *info;
};

/* The values of the various regulator constraints are obviously dependent
 * on exactly what is wired to each ldo.  Unfortunately this information is
 * not generally available.  More information has been requested from Xbow
 * but as of yet they haven't been forthcoming.
 *
 * Some of these are clearly Stargate 2 related (no way of plugging
 * in an lcd on the IM2 for example!).
 */

struct axp_consumer_supply {
	char supply[20];    /* consumer supply - e.g. "vcc" */
};

#define AXP_LDOIO_ID_START      30
#define AXP_DCDC_ID_START       40

#define AXP_LDOSN(_pmic, _id, min, max, step1, vreg, shift, nbits, ereg,\
	emask, enval, disval, switch_vol, step2, new_level,\
	mode_addr, freq_addr, dvm_ereg, dvm_ebit, dvm_flag, sname)\
{                                                      \
	.desc   = {                                        \
		.name   = #_pmic"_LDO" #_id,                   \
		.supply_name = sname,                          \
		.type   = REGULATOR_VOLTAGE,                   \
		.id = _pmic##_ID_LDO##_id,                     \
		.n_voltages = (step1) ? ((switch_vol) ?        \
				((new_level) ? ((switch_vol - min) / step1 + \
				(max - new_level) / step2 + 2)         \
				: ((switch_vol - min) / step1 +        \
				(max - switch_vol) / step2 + 1)) :     \
				((max - min) / step1 + 1)) : 1,        \
		.owner  = THIS_MODULE,                         \
		.enable_reg  = _pmic##_##ereg,                 \
		.enable_mask = emask,                          \
	},                                                 \
	.min_uv     = (min) * 1000,                        \
	.max_uv     = (max) * 1000,                        \
	.enable_val  = enval,                              \
	.disable_val = disval,                             \
	.step1_uv   = (step1) * 1000,                      \
	.vol_reg    = _pmic##_##vreg,                      \
	.vol_shift  = (shift),                             \
	.vol_nbits  = (nbits),                             \
	.switch_uv  = (switch_vol)*1000,                   \
	.step2_uv   = (step2)*1000,                        \
	.new_level_uv   = (new_level)*1000,                \
	.mode_reg   = mode_addr,                           \
	.freq_reg   = freq_addr,                           \
	.dvm_enable_reg = dvm_ereg,                        \
	.dvm_enable_bit = dvm_ebit,                        \
	.dvm_finish_flag = dvm_flag,                       \
}

#define AXP_LDO(_pmic, _id, min, max, step1, vreg, shift, nbits, ereg,\
	emask, enval, disval, switch_vol, step2, new_level,\
	mode_addr, freq_addr, dvm_ereg, dvm_ebit, dvm_flag)\
	AXP_LDOSN(_pmic, _id, min, max, step1, vreg, shift, nbits, ereg,\
	emask, enval, disval, switch_vol, step2, new_level,\
	mode_addr, freq_addr, dvm_ereg, dvm_ebit, dvm_flag, "")


#define AXP_DCDC(_pmic, _id, min, max, step1, vreg, shift, nbits, ereg,\
	emask, enval, disval, switch_vol, step2, new_level,\
	mode_addr, mode_bit, freq_addr, dvm_ereg, dvm_ebit, dvm_flag)\
{                                                      \
	.desc   = {                                        \
		.name   = #_pmic"_DCDC" #_id,                  \
		.type   = REGULATOR_VOLTAGE,                   \
		.id = _pmic##_ID_DCDC##_id,                    \
		.n_voltages = (step1) ? ((switch_vol) ?        \
				((new_level) ? ((switch_vol - min) / step1 + \
				(max - new_level) / step2 + 2)         \
				: ((switch_vol - min) / step1 +        \
				(max - switch_vol) / step2 + 1)) :     \
				((max - min) / step1 + 1)) : 1,        \
		.owner  = THIS_MODULE,                         \
		.enable_reg  = _pmic##_##ereg,                 \
		.enable_mask = emask,                          \
	},                                                 \
	.min_uv     = (min) * 1000,                        \
	.max_uv     = (max) * 1000,                        \
	.enable_val  = enval,                              \
	.disable_val = disval,                             \
	.step1_uv   = (step1) * 1000,                      \
	.vol_reg    = _pmic##_##vreg,                      \
	.vol_shift  = (shift),                             \
	.vol_nbits  = (nbits),                             \
	.switch_uv  = (switch_vol)*1000,                   \
	.step2_uv   = (step2)*1000,                        \
	.new_level_uv   = (new_level)*1000,                \
	.mode_reg   = mode_addr,                           \
	.mode_mask  = mode_bit,                            \
	.freq_reg   = freq_addr,                           \
	.dvm_enable_reg = dvm_ereg,                        \
	.dvm_enable_bit = dvm_ebit,                        \
	.dvm_finish_flag = dvm_flag,                       \
}


#define AXP_SW(_pmic, _id, min, max, step1, vreg, shift, nbits, ereg,\
	emask, enval, disval, switch_vol, step2, new_level,\
	mode_addr, freq_addr, dvm_ereg, dvm_ebit, dvm_flag)\
{                                                      \
	.desc   = {                                        \
		.name   = #_pmic"_SW" #_id,                    \
		.type   = REGULATOR_VOLTAGE,                   \
		.id = _pmic##_ID_SW##_id,                      \
		.n_voltages = (step1) ? ((switch_vol) ?        \
				((new_level) ? ((switch_vol - min) / step1 + \
				(max - new_level) / step2 + 2)         \
				: ((switch_vol - min) / step1 +        \
				(max - switch_vol) / step2 + 1)) :     \
				((max - min) / step1 + 1)) : 1,        \
		.owner  = THIS_MODULE,                         \
		.enable_reg  = _pmic##_##ereg,                 \
		.enable_mask = emask,                          \
	},                                                 \
	.min_uv     = (min) * 1000,                        \
	.max_uv     = (max) * 1000,                        \
	.step1_uv   = (step1) * 1000,                      \
	.enable_val  = enval,                              \
	.disable_val = disval,                             \
	.vol_reg    = _pmic##_##vreg,                      \
	.vol_shift  = (shift),                             \
	.vol_nbits  = (nbits),                             \
	.switch_uv  = (switch_vol)*1000,                   \
	.step2_uv   = (step2)*1000,                        \
	.new_level_uv   = (new_level)*1000,                \
	.mode_reg   = mode_addr,                           \
	.freq_reg   = freq_addr,                           \
	.dvm_enable_reg = dvm_ereg,                        \
	.dvm_enable_bit = dvm_ebit,                        \
	.dvm_finish_flag = dvm_flag,                       \
}

#define AXP_LDO_SEL(_pmic, _id, min, max, vreg, shift, nbits, ereg,\
	emask, enval, disval, table_name,\
	mode_addr, freq_addr, dvm_ereg, dvm_ebit, dvm_flag)\
{                                                      \
	.desc   = {                                        \
		.name   = #_pmic"_LDO" #_id,                   \
		.type   = REGULATOR_VOLTAGE,                   \
		.id = _pmic##_ID_LDO##_id,                     \
		.n_voltages = ARRAY_SIZE(table_name##_table),  \
		.owner  = THIS_MODULE,                         \
		.enable_reg  = _pmic##_##ereg,                 \
		.enable_mask = emask,                          \
	},                                                 \
	.min_uv     = (min) * 1000,                        \
	.max_uv     = (max) * 1000,                        \
	.enable_val  = enval,                              \
	.disable_val = disval,                             \
	.vol_reg    = _pmic##_##vreg,                      \
	.vol_shift  = (shift),                             \
	.vol_nbits  = (nbits),                             \
	.mode_reg   = mode_addr,                           \
	.freq_reg   = freq_addr,                           \
	.dvm_enable_reg = dvm_ereg,                        \
	.dvm_enable_bit = dvm_ebit,                        \
	.dvm_finish_flag = dvm_flag,                       \
	.vtable         = (int *)&table_name##_table,      \
}

#define AXP_DCDC_SEL(_pmic, _id, min, max, vreg, shift, nbits, ereg,\
	emask, enval, disval, table_name,\
	mode_addr, mode_bit, freq_addr, dvm_ereg, dvm_ebit, dvm_flag)\
{                                                      \
	.desc   = {                                        \
		.name   = #_pmic"_DCDC" #_id,                  \
		.type   = REGULATOR_VOLTAGE,                   \
		.id = _pmic##_ID_DCDC##_id,                    \
		.n_voltages = ARRAY_SIZE(table_name##_table),  \
		.owner  = THIS_MODULE,                         \
		.enable_reg  = _pmic##_##ereg,                 \
		.enable_mask = emask,                          \
	},                                                 \
	.min_uv     = (min) * 1000,                        \
	.max_uv     = (max) * 1000,                        \
	.enable_val  = enval,                              \
	.disable_val = disval,                             \
	.vol_reg    = _pmic##_##vreg,                      \
	.vol_shift  = (shift),                             \
	.vol_nbits  = (nbits),                             \
	.mode_reg   = mode_addr,                           \
	.mode_mask  = mode_bit,                            \
	.freq_reg   = freq_addr,                           \
	.dvm_enable_reg = dvm_ereg,                        \
	.dvm_enable_bit = dvm_ebit,                        \
	.dvm_finish_flag = dvm_flag,                       \
	.vtable         = (int *)&table_name##_table,      \
}


#define AXP_SW_SEL(_pmic, _id, min, max, vreg, shift, nbits, ereg,\
	emask, enval, disval, table_name,\
	mode_addr, freq_addr, dvm_ereg, dvm_ebit, dvm_flag)\
{                                                      \
	.desc   = {                                        \
		.name   = #_pmic"_SW" #_id,                    \
		.type   = REGULATOR_VOLTAGE,                   \
		.id = _pmic##_ID_SW##_id,                      \
		.n_voltages = ARRAY_SIZE(table_name##_table),  \
		.owner  = THIS_MODULE,                         \
		.enable_reg  = _pmic##_##ereg,                 \
		.enable_mask = emask,                          \
	},                                                 \
	.min_uv     = (min) * 1000,                        \
	.max_uv     = (max) * 1000,                        \
	.enable_val  = enval,                              \
	.disable_val = disval,                             \
	.vol_reg    = _pmic##_##vreg,                      \
	.vol_shift  = (shift),                             \
	.vol_nbits  = (nbits),                             \
	.mode_reg   = mode_addr,                           \
	.freq_reg   = freq_addr,                           \
	.dvm_enable_reg = dvm_ereg,                        \
	.dvm_enable_bit = dvm_ebit,                        \
	.dvm_finish_flag = dvm_flag,                       \
	.vtable          = (int *)&table_name##_table,     \
}

#define AXP_REGULATOR_ATTR(_name)                          \
{                                                          \
	.attr = { .name = #_name, .mode = 0644 },          \
	.show =  _name##_show,                             \
	.store = _name##_store,                            \
}

struct axp_regulator_info {
	struct regulator_desc desc;
	struct axp_regmap *regmap;
	s32 min_uv;
	s32 max_uv;
	s32 enable_val;
	s32 disable_val;
	s32 step1_uv;
	s32 vol_reg;
	s32 vol_shift;
	s32 vol_nbits;
	s32 switch_uv;
	s32 step2_uv;
	s32 new_level_uv;
	s32 mode_reg;
	s32 mode_mask;
	s32 freq_reg;
	s32 dvm_enable_reg;
	s32 dvm_enable_bit;
	s32 dvm_finish_flag;
	s32 *vtable;
	s32 pmu_num;
};

struct regulator {
	struct device *dev;
	struct list_head list;
	unsigned int always_on:1;
	unsigned int bypass:1;
	int uA_load;
	int min_uV;
	int max_uV;
	char *supply_name;
	struct device_attribute dev_attr;
	struct regulator_dev *rdev;
	struct dentry *debugfs;
};

typedef struct {
	unsigned int mem_data;
	char id_name[20];
} axp_mem_data_t;

struct regulator_dev *axp_regulator_register(struct device *dev,
				struct axp_regmap *regmap,
				struct regulator_init_data *init_data,
				struct axp_regulator_info *info);
struct regulator_dev *axp_regulator_sel_register(struct device *dev,
				struct axp_regmap *regmap,
				struct regulator_init_data *init_data,
				struct axp_regulator_info *info);
void axp_regulator_unregister(struct regulator_dev *rdev);
s32 axp_regulator_dt_parse(struct device_node *node,
				struct regulator_init_data *axp_init_data,
				s32 (*get_dep_cb)(const char *));
int axp_get_ldo_count(struct device_node *node, u32 *ldo_count);
int axp_mem_regu_init(struct device_node *node,
				axp_mem_data_t *regu_list, u32 ldo_count);
int axp_regulator_create_attrs(struct device *dev);

#endif /* AXP_REGULATOR_H */
