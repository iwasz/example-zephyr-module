/* lsm6ds3_spi.c - SPI routines for LSM6DS3 driver
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

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

#define LSM6DS3_SPI_READ (1 << 7)

LOG_MODULE_DECLARE (LSM6DS3, CONFIG_SENSOR_LOG_LEVEL);

static int lsm6ds3_raw_read (const struct device *dev, uint8_t reg_addr, uint8_t *value, uint8_t len)
{
        struct lsm6ds3_data *data = dev->data;
        const struct lsm6ds3_config *cfg = dev->config;
        const struct spi_config *spi_cfg = &cfg->bus_cfg.spi_cfg->spi_conf;
        uint8_t buffer_tx[2] = {reg_addr | LSM6DS3_SPI_READ, 0};
        const struct spi_buf tx_buf = {
                .buf = buffer_tx,
                .len = 2,
        };
        const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
        const struct spi_buf rx_buf[2] = {{
                                                  .buf = NULL,
                                                  .len = 1,
                                          },
                                          {
                                                  .buf = value,
                                                  .len = len,
                                          }};
        const struct spi_buf_set rx = {.buffers = rx_buf, .count = 2};

        if (len > 64) {
                return -EIO;
        }

        if (spi_transceive (data->bus, spi_cfg, &tx, &rx)) {
                return -EIO;
        }

        return 0;
}

static int lsm6ds3_raw_write (const struct device *dev, uint8_t reg_addr, uint8_t *value, uint8_t len)
{
        struct lsm6ds3_data *data = dev->data;
        const struct lsm6ds3_config *cfg = dev->config;
        const struct spi_config *spi_cfg = &cfg->bus_cfg.spi_cfg->spi_conf;
        uint8_t buffer_tx[1] = {reg_addr & ~LSM6DS3_SPI_READ};
        const struct spi_buf tx_buf[2] = {{
                                                  .buf = buffer_tx,
                                                  .len = 1,
                                          },
                                          {
                                                  .buf = value,
                                                  .len = len,
                                          }};
        const struct spi_buf_set tx = {.buffers = tx_buf, .count = 2};

        if (len > 64) {
                return -EIO;
        }

        if (spi_write (data->bus, spi_cfg, &tx)) {
                return -EIO;
        }

        return 0;
}

static int lsm6ds3_spi_read_data (const struct device *dev, uint8_t reg_addr, uint8_t *value, uint8_t len)
{
        return lsm6ds3_raw_read (dev, reg_addr, value, len);
}

static int lsm6ds3_spi_write_data (const struct device *dev, uint8_t reg_addr, uint8_t *value, uint8_t len)
{
        return lsm6ds3_raw_write (dev, reg_addr, value, len);
}

static int lsm6ds3_spi_read_reg (const struct device *dev, uint8_t reg_addr, uint8_t *value)
{
        return lsm6ds3_raw_read (dev, reg_addr, value, 1);
}

static int lsm6ds3_spi_update_reg (const struct device *dev, uint8_t reg_addr, uint8_t mask, uint8_t value)
{
        uint8_t tmp_val;

        lsm6ds3_raw_read (dev, reg_addr, &tmp_val, 1);
        tmp_val = (tmp_val & ~mask) | (value & mask);

        return lsm6ds3_raw_write (dev, reg_addr, &tmp_val, 1);
}

static const struct lsm6ds3_transfer_function lsm6ds3_spi_transfer_fn = {
        .read_data = lsm6ds3_spi_read_data,
        .write_data = lsm6ds3_spi_write_data,
        .read_reg = lsm6ds3_spi_read_reg,
        .update_reg = lsm6ds3_spi_update_reg,
};

int lsm6ds3_spi_init (const struct device *dev)
{
        struct lsm6ds3_data *data = dev->data;
        const struct lsm6ds3_config *cfg = dev->config;
        const struct lsm6ds3_spi_cfg *spi_cfg = cfg->bus_cfg.spi_cfg;

        data->hw_tf = &lsm6ds3_spi_transfer_fn;

        if (spi_cfg->cs_gpios_label != NULL) {

                /* handle SPI CS thru GPIO if it is the case */
                data->cs_ctrl.gpio_dev = device_get_binding (spi_cfg->cs_gpios_label);
                if (!data->cs_ctrl.gpio_dev) {
                        LOG_ERR ("Unable to get GPIO SPI CS device");
                        return -ENODEV;
                }

                LOG_DBG ("SPI GPIO CS configured on %s:%u", spi_cfg->cs_gpios_label, data->cs_ctrl.gpio_pin);
        }

        return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
