/* lsm6ds3_i2c.c - I2C routines for LSM6DS3 driver
 */

/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_lsm6ds3

#include <logging/log.h>
#include <string.h>

#include "lsm6ds3.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

LOG_MODULE_DECLARE (LSM6DS3, CONFIG_SENSOR_LOG_LEVEL);

static int lsm6ds3_i2c_read_data (const struct device *dev, uint8_t reg_addr, uint8_t *value, uint8_t len)
{
        struct lsm6ds3_data *data = dev->data;
        const struct lsm6ds3_config *cfg = dev->config;

        return i2c_burst_read (data->bus, cfg->bus_cfg.i2c_slv_addr, reg_addr, value, len);
}

static int lsm6ds3_i2c_write_data (const struct device *dev, uint8_t reg_addr, uint8_t *value, uint8_t len)
{
        struct lsm6ds3_data *data = dev->data;
        const struct lsm6ds3_config *cfg = dev->config;

        return i2c_burst_write (data->bus, cfg->bus_cfg.i2c_slv_addr, reg_addr, value, len);
}

static int lsm6ds3_i2c_read_reg (const struct device *dev, uint8_t reg_addr, uint8_t *value)
{
        struct lsm6ds3_data *data = dev->data;
        const struct lsm6ds3_config *cfg = dev->config;

        return i2c_reg_read_byte (data->bus, cfg->bus_cfg.i2c_slv_addr, reg_addr, value);
}

static int lsm6ds3_i2c_update_reg (const struct device *dev, uint8_t reg_addr, uint8_t mask, uint8_t value)
{
        struct lsm6ds3_data *data = dev->data;
        const struct lsm6ds3_config *cfg = dev->config;

        return i2c_reg_update_byte (data->bus, cfg->bus_cfg.i2c_slv_addr, reg_addr, mask, value);
}

static const struct lsm6ds3_transfer_function lsm6ds3_i2c_transfer_fn = {
        .read_data = lsm6ds3_i2c_read_data,
        .write_data = lsm6ds3_i2c_write_data,
        .read_reg = lsm6ds3_i2c_read_reg,
        .update_reg = lsm6ds3_i2c_update_reg,
};

int lsm6ds3_i2c_init (const struct device *dev)
{
        struct lsm6ds3_data *data = dev->data;

        data->hw_tf = &lsm6ds3_i2c_transfer_fn;

        return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
