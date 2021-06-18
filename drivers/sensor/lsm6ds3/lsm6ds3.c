/* lsm6ds3.c - Driver for LSM6DS3 accelerometer, gyroscope and
 * temperature sensor
 */

/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_lsm6ds3

#include <device.h>
#include <drivers/sensor.h>
#include <init.h>
#include <kernel.h>
#include <logging/log.h>
#include <string.h>
#include <sys/__assert.h>
#include <sys/byteorder.h>

#include "lsm6ds3.h"

LOG_MODULE_REGISTER (LSM6DS3, CONFIG_SENSOR_LOG_LEVEL);

static const uint16_t lsm6ds3_odr_map[] = {0, 12, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660};

#if defined(LSM6DS3_ACCEL_ODR_RUNTIME) || defined(LSM6DS3_GYRO_ODR_RUNTIME)
static int lsm6ds3_freq_to_odr_val (uint16_t freq)
{
        size_t i;

        for (i = 0; i < ARRAY_SIZE (lsm6ds3_odr_map); i++) {
                if (freq == lsm6ds3_odr_map[i]) {
                        return i;
                }
        }

        return -EINVAL;
}
#endif

static int lsm6ds3_odr_to_freq_val (uint16_t odr)
{
        /* for valid index, return value from map */
        if (odr < ARRAY_SIZE (lsm6ds3_odr_map)) {
                return lsm6ds3_odr_map[odr];
        }

        /* invalid index, return last entry */
        return lsm6ds3_odr_map[ARRAY_SIZE (lsm6ds3_odr_map) - 1];
}

#ifdef LSM6DS3_ACCEL_FS_RUNTIME
static const uint16_t lsm6ds3_accel_fs_map[] = {2, 16, 4, 8};
static const uint16_t lsm6ds3_accel_fs_sens[] = {1, 8, 2, 4};

static int lsm6ds3_accel_range_to_fs_val (int32_t range)
{
        size_t i;

        for (i = 0; i < ARRAY_SIZE (lsm6ds3_accel_fs_map); i++) {
                if (range == lsm6ds3_accel_fs_map[i]) {
                        return i;
                }
        }

        return -EINVAL;
}
#endif

#ifdef LSM6DS3_GYRO_FS_RUNTIME
static const uint16_t lsm6ds3_gyro_fs_map[] = {245, 500, 1000, 2000, 125};
static const uint16_t lsm6ds3_gyro_fs_sens[] = {2, 4, 8, 16, 1};

static int lsm6ds3_gyro_range_to_fs_val (int32_t range)
{
        size_t i;

        for (i = 0; i < ARRAY_SIZE (lsm6ds3_gyro_fs_map); i++) {
                if (range == lsm6ds3_gyro_fs_map[i]) {
                        return i;
                }
        }

        return -EINVAL;
}
#endif

static inline int lsm6ds3_reboot (const struct device *dev)
{
        struct lsm6ds3_data *data = dev->data;

        if (data->hw_tf->update_reg (dev, LSM6DS3_REG_CTRL3_C, LSM6DS3_MASK_CTRL3_C_BOOT, 1 << LSM6DS3_SHIFT_CTRL3_C_BOOT) < 0) {
                return -EIO;
        }

        /* Wait sensor turn-on time as per datasheet */
        k_busy_wait (USEC_PER_MSEC * 35U);

        return 0;
}

static int lsm6ds3_accel_set_fs_raw (const struct device *dev, uint8_t fs)
{
        struct lsm6ds3_data *data = dev->data;

        if (data->hw_tf->update_reg (dev, LSM6DS3_REG_CTRL1_XL, LSM6DS3_MASK_CTRL1_XL_FS_XL, fs << LSM6DS3_SHIFT_CTRL1_XL_FS_XL) < 0) {
                return -EIO;
        }

        data->accel_fs = fs;

        return 0;
}

static int lsm6ds3_accel_set_odr_raw (const struct device *dev, uint8_t odr)
{
        struct lsm6ds3_data *data = dev->data;

        if (data->hw_tf->update_reg (dev, LSM6DS3_REG_CTRL1_XL, LSM6DS3_MASK_CTRL1_XL_ODR_XL, odr << LSM6DS3_SHIFT_CTRL1_XL_ODR_XL) < 0) {
                return -EIO;
        }

        data->accel_freq = lsm6ds3_odr_to_freq_val (odr);

        // // Turning the LPF2 (Low Pass Filter 2) on LPF2_XL_EN = 1.
        // if (data->hw_tf->update_reg (dev, LSM6DS3_REG_CTRL8_XL, LSM6DS3_MASK_CTRL8_LPF2_XL_EN, 1 << LSM6DS3_SHIFT_CTRL8_LPF2_XL_EN) < 0) {
        //         return -EIO;
        // }

        // if (data->hw_tf->update_reg (dev, LSM6DS3_REG_CTRL8_XL, LSM6DS3_MASK_CTRL8_HP_SLOPE_XL_EN, 1 << LSM6DS3_SHIFT_CTRL8_HP_SLOPE_XL_EN)
        //     < 0) {
        //         return -EIO;
        // }

        // if (data->hw_tf->update_reg (dev, LSM6DS3_REG_CTRL8_XL, LSM6DS3_MASK_CTRL8_HPCF_XL, 0b00 << LSM6DS3_SHIFT_CTRL8_HPCF_XL) < 0) {
        //         return -EIO;
        // }

        return 0;
}

static int lsm6ds3_gyro_set_fs_raw (const struct device *dev, uint8_t fs)
{
        struct lsm6ds3_data *data = dev->data;

        if (fs == GYRO_FULLSCALE_125) {
                if (data->hw_tf->update_reg (dev, LSM6DS3_REG_CTRL2_G, LSM6DS3_MASK_CTRL2_FS125, 1 << LSM6DS3_SHIFT_CTRL2_FS125) < 0) {
                        return -EIO;
                }
        }
        else {
                if (data->hw_tf->update_reg (dev, LSM6DS3_REG_CTRL2_G, LSM6DS3_MASK_CTRL2_G_FS_G, fs << LSM6DS3_SHIFT_CTRL2_G_FS_G) < 0) {
                        return -EIO;
                }
        }

        return 0;
}

static int lsm6ds3_gyro_set_odr_raw (const struct device *dev, uint8_t odr)
{
        struct lsm6ds3_data *data = dev->data;

        if (data->hw_tf->update_reg (dev, LSM6DS3_REG_CTRL2_G, LSM6DS3_MASK_CTRL2_G_ODR_G, odr << LSM6DS3_SHIFT_CTRL2_G_ODR_G) < 0) {
                return -EIO;
        }

        return 0;
}

#ifdef LSM6DS3_ACCEL_ODR_RUNTIME
static int lsm6ds3_accel_odr_set (const struct device *dev, uint16_t freq)
{
        int odr;

        odr = lsm6ds3_freq_to_odr_val (freq);
        if (odr < 0) {
                return odr;
        }

        if (lsm6ds3_accel_set_odr_raw (dev, odr) < 0) {
                LOG_DBG ("failed to set accelerometer sampling rate");
                return -EIO;
        }

        return 0;
}
#endif

#ifdef LSM6DS3_ACCEL_FS_RUNTIME
static int lsm6ds3_accel_range_set (const struct device *dev, int32_t range)
{
        int fs;
        struct lsm6ds3_data *data = dev->data;

        fs = lsm6ds3_accel_range_to_fs_val (range);
        if (fs < 0) {
                return fs;
        }

        if (lsm6ds3_accel_set_fs_raw (dev, fs) < 0) {
                LOG_DBG ("failed to set accelerometer full-scale");
                return -EIO;
        }

        data->accel_sensitivity = (float)(lsm6ds3_accel_fs_sens[fs] * SENSI_GRAIN_XL);
        return 0;
}
#endif

static int lsm6ds3_accel_config (const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
{
        switch (attr) {
#ifdef LSM6DS3_ACCEL_FS_RUNTIME
        case SENSOR_ATTR_FULL_SCALE:
                return lsm6ds3_accel_range_set (dev, sensor_ms2_to_g (val));
#endif
#ifdef LSM6DS3_ACCEL_ODR_RUNTIME
        case SENSOR_ATTR_SAMPLING_FREQUENCY:
                return lsm6ds3_accel_odr_set (dev, val->val1);
#endif
        default:
                LOG_DBG ("Accel attribute not supported.");
                return -ENOTSUP;
        }

        return 0;
}

#ifdef LSM6DS3_GYRO_ODR_RUNTIME
static int lsm6ds3_gyro_odr_set (const struct device *dev, uint16_t freq)
{
        int odr;

        odr = lsm6ds3_freq_to_odr_val (freq);
        if (odr < 0) {
                return odr;
        }

        if (lsm6ds3_gyro_set_odr_raw (dev, odr) < 0) {
                LOG_DBG ("failed to set gyroscope sampling rate");
                return -EIO;
        }

        return 0;
}
#endif

#ifdef LSM6DS3_GYRO_FS_RUNTIME
static int lsm6ds3_gyro_range_set (const struct device *dev, int32_t range)
{
        int fs;
        struct lsm6ds3_data *data = dev->data;

        fs = lsm6ds3_gyro_range_to_fs_val (range);
        if (fs < 0) {
                return fs;
        }

        if (lsm6ds3_gyro_set_fs_raw (dev, fs) < 0) {
                LOG_DBG ("failed to set gyroscope full-scale");
                return -EIO;
        }

        data->gyro_sensitivity = (float)(lsm6ds3_gyro_fs_sens[fs] * SENSI_GRAIN_G);
        return 0;
}
#endif

static int lsm6ds3_gyro_config (const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
{
        switch (attr) {
#ifdef LSM6DS3_GYRO_FS_RUNTIME
        case SENSOR_ATTR_FULL_SCALE:
                return lsm6ds3_gyro_range_set (dev, sensor_rad_to_degrees (val));
#endif
#ifdef LSM6DS3_GYRO_ODR_RUNTIME
        case SENSOR_ATTR_SAMPLING_FREQUENCY:
                return lsm6ds3_gyro_odr_set (dev, val->val1);
#endif
        default:
                LOG_DBG ("Gyro attribute not supported.");
                return -ENOTSUP;
        }

        return 0;
}

static int lsm6ds3_attr_set (const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
{
        switch (chan) {
        case SENSOR_CHAN_ACCEL_XYZ:
                return lsm6ds3_accel_config (dev, chan, attr, val);
        case SENSOR_CHAN_GYRO_XYZ:
                return lsm6ds3_gyro_config (dev, chan, attr, val);
        default:
                LOG_WRN ("attr_set() not supported on this channel.");
                return -ENOTSUP;
        }

        return 0;
}

static int lsm6ds3_sample_fetch_accel (const struct device *dev)
{
        struct lsm6ds3_data *data = dev->data;
        uint8_t buf[6];

        if (data->hw_tf->read_data (dev, LSM6DS3_REG_OUTX_L_XL, buf, sizeof (buf)) < 0) {
                LOG_DBG ("failed to read sample");
                return -EIO;
        }

        data->accel_sample_x = (int16_t)((uint16_t)(buf[0]) | ((uint16_t)(buf[1]) << 8));
        data->accel_sample_y = (int16_t)((uint16_t)(buf[2]) | ((uint16_t)(buf[3]) << 8));
        data->accel_sample_z = (int16_t)((uint16_t)(buf[4]) | ((uint16_t)(buf[5]) << 8));

        return 0;
}

static int lsm6ds3_sample_fetch_gyro (const struct device *dev)
{
        struct lsm6ds3_data *data = dev->data;
        uint8_t buf[6];

        if (data->hw_tf->read_data (dev, LSM6DS3_REG_OUTX_L_G, buf, sizeof (buf)) < 0) {
                LOG_DBG ("failed to read sample");
                return -EIO;
        }

        data->gyro_sample_x = (int16_t)((uint16_t)(buf[0]) | ((uint16_t)(buf[1]) << 8));
        data->gyro_sample_y = (int16_t)((uint16_t)(buf[2]) | ((uint16_t)(buf[3]) << 8));
        data->gyro_sample_z = (int16_t)((uint16_t)(buf[4]) | ((uint16_t)(buf[5]) << 8));

        return 0;
}

#if defined(CONFIG_LSM6DS3_ENABLE_TEMP)
static int lsm6ds3_sample_fetch_temp (const struct device *dev)
{
        struct lsm6ds3_data *data = dev->data;
        uint8_t buf[2];

        if (data->hw_tf->read_data (dev, LSM6DS3_REG_OUT_TEMP_L, buf, sizeof (buf)) < 0) {
                LOG_DBG ("failed to read sample");
                return -EIO;
        }

        data->temp_sample = (int16_t)((uint16_t)(buf[0]) | ((uint16_t)(buf[1]) << 8));

        return 0;
}
#endif

#if defined(CONFIG_LSM6DS3_EXT0_LIS2MDL)
static int lsm6ds3_sample_fetch_magn (const struct device *dev)
{
        struct lsm6ds3_data *data = dev->data;
        uint8_t buf[6];

        if (lsm6ds3_shub_read_external_chip (dev, buf, sizeof (buf)) < 0) {
                LOG_DBG ("failed to read ext mag sample");
                return -EIO;
        }

        data->magn_sample_x = (int16_t)((uint16_t)(buf[0]) | ((uint16_t)(buf[1]) << 8));
        data->magn_sample_y = (int16_t)((uint16_t)(buf[2]) | ((uint16_t)(buf[3]) << 8));
        data->magn_sample_z = (int16_t)((uint16_t)(buf[4]) | ((uint16_t)(buf[5]) << 8));

        return 0;
}
#endif
#if defined(CONFIG_LSM6DS3_EXT0_LPS22HB)
static int lsm6ds3_sample_fetch_press (const struct device *dev)
{
        struct lsm6ds3_data *data = dev->data;
        uint8_t buf[5];

        if (lsm6ds3_shub_read_external_chip (dev, buf, sizeof (buf)) < 0) {
                LOG_DBG ("failed to read ext press sample");
                return -EIO;
        }

        data->sample_press = (int32_t)((uint32_t)(buf[0]) | ((uint32_t)(buf[1]) << 8) | ((uint32_t)(buf[2]) << 16));
        data->sample_temp = (int16_t)((uint16_t)(buf[3]) | ((uint16_t)(buf[4]) << 8));

        return 0;
}
#endif

static int lsm6ds3_sample_fetch (const struct device *dev, enum sensor_channel chan)
{
        switch (chan) {
        case SENSOR_CHAN_ACCEL_XYZ:
                lsm6ds3_sample_fetch_accel (dev);
                break;
        case SENSOR_CHAN_GYRO_XYZ:
                lsm6ds3_sample_fetch_gyro (dev);
                break;
#if defined(CONFIG_LSM6DS3_ENABLE_TEMP)
        case SENSOR_CHAN_DIE_TEMP:
                lsm6ds3_sample_fetch_temp (dev);
                break;
#endif
#if defined(CONFIG_LSM6DS3_EXT0_LIS2MDL)
        case SENSOR_CHAN_MAGN_XYZ:
                lsm6ds3_sample_fetch_magn (dev);
                break;
#endif
#if defined(CONFIG_LSM6DS3_EXT0_LPS22HB)
        case SENSOR_CHAN_AMBIENT_TEMP:
        case SENSOR_CHAN_PRESS:
                lsm6ds3_sample_fetch_press (dev);
                break;
#endif
        case SENSOR_CHAN_ALL:
                lsm6ds3_sample_fetch_accel (dev);
                lsm6ds3_sample_fetch_gyro (dev);
#if defined(CONFIG_LSM6DS3_ENABLE_TEMP)
                lsm6ds3_sample_fetch_temp (dev);
#endif
#if defined(CONFIG_LSM6DS3_EXT0_LIS2MDL)
                lsm6ds3_sample_fetch_magn (dev);
#endif
#if defined(CONFIG_LSM6DS3_EXT0_LPS22HB)
                lsm6ds3_sample_fetch_press (dev);
#endif
                break;
        default:
                return -ENOTSUP;
        }

        return 0;
}

static inline void lsm6ds3_accel_convert (struct sensor_value *val, int raw_val, float sensitivity)
{
        double dval;

        /* Sensitivity is exposed in mg/LSB */
        /* Convert to m/s^2 */
        dval = (double)(raw_val)*sensitivity * SENSOR_G_DOUBLE / 1000;
        val->val1 = (int32_t)dval;
        val->val2 = (((int32_t)(dval * 1000)) % 1000) * 1000;
}

static inline int lsm6ds3_accel_get_channel (enum sensor_channel chan, struct sensor_value *val, struct lsm6ds3_data *data, float sensitivity)
{
        switch (chan) {
        case SENSOR_CHAN_ACCEL_X:
                lsm6ds3_accel_convert (val, data->accel_sample_x, sensitivity);
                break;
        case SENSOR_CHAN_ACCEL_Y:
                lsm6ds3_accel_convert (val, data->accel_sample_y, sensitivity);
                break;
        case SENSOR_CHAN_ACCEL_Z:
                lsm6ds3_accel_convert (val, data->accel_sample_z, sensitivity);
                break;
        case SENSOR_CHAN_ACCEL_XYZ:
                lsm6ds3_accel_convert (val, data->accel_sample_x, sensitivity);
                lsm6ds3_accel_convert (val + 1, data->accel_sample_y, sensitivity);
                lsm6ds3_accel_convert (val + 2, data->accel_sample_z, sensitivity);
                break;
        default:
                return -ENOTSUP;
        }

        return 0;
}

static int lsm6ds3_accel_channel_get (enum sensor_channel chan, struct sensor_value *val, struct lsm6ds3_data *data)
{
        return lsm6ds3_accel_get_channel (chan, val, data, data->accel_sensitivity);
}

static inline void lsm6ds3_gyro_convert (struct sensor_value *val, int raw_val, float sensitivity)
{
        double dval;

        /* Sensitivity is exposed in mdps/LSB */
        /* Convert to rad/s */
        dval = (double)(raw_val * sensitivity * SENSOR_DEG2RAD_DOUBLE / 1000);
        val->val1 = (int32_t)dval;
        val->val2 = (((int32_t)(dval * 1000)) % 1000) * 1000;
}

static inline int lsm6ds3_gyro_get_channel (enum sensor_channel chan, struct sensor_value *val, struct lsm6ds3_data *data, float sensitivity)
{
        switch (chan) {
        case SENSOR_CHAN_GYRO_X:
                lsm6ds3_gyro_convert (val, data->gyro_sample_x, sensitivity);
                break;
        case SENSOR_CHAN_GYRO_Y:
                lsm6ds3_gyro_convert (val, data->gyro_sample_y, sensitivity);
                break;
        case SENSOR_CHAN_GYRO_Z:
                lsm6ds3_gyro_convert (val, data->gyro_sample_z, sensitivity);
                break;
        case SENSOR_CHAN_GYRO_XYZ:
                lsm6ds3_gyro_convert (val, data->gyro_sample_x, sensitivity);
                lsm6ds3_gyro_convert (val + 1, data->gyro_sample_y, sensitivity);
                lsm6ds3_gyro_convert (val + 2, data->gyro_sample_z, sensitivity);
                break;
        default:
                return -ENOTSUP;
        }

        return 0;
}

static int lsm6ds3_gyro_channel_get (enum sensor_channel chan, struct sensor_value *val, struct lsm6ds3_data *data)
{
        return lsm6ds3_gyro_get_channel (chan, val, data, LSM6DS3_DEFAULT_GYRO_SENSITIVITY);
}

#if defined(CONFIG_LSM6DS3_ENABLE_TEMP)
static void lsm6ds3_gyro_channel_get_temp (struct sensor_value *val, struct lsm6ds3_data *data)
{
        /* val = temp_sample / 256 + 25 */
        val->val1 = data->temp_sample / 256 + 25;
        val->val2 = (data->temp_sample % 256) * (1000000 / 256);
}
#endif

#if defined(CONFIG_LSM6DS3_EXT0_LIS2MDL)
static inline void lsm6ds3_magn_convert (struct sensor_value *val, int raw_val, float sensitivity)
{
        double dval;

        /* Sensitivity is exposed in mgauss/LSB */
        dval = (double)(raw_val * sensitivity);
        val->val1 = (int32_t)dval / 1000000;
        val->val2 = (int32_t)dval % 1000000;
}

static inline int lsm6ds3_magn_get_channel (enum sensor_channel chan, struct sensor_value *val, struct lsm6ds3_data *data)
{
        switch (chan) {
        case SENSOR_CHAN_MAGN_X:
                lsm6ds3_magn_convert (val, data->magn_sample_x, data->magn_sensitivity);
                break;
        case SENSOR_CHAN_MAGN_Y:
                lsm6ds3_magn_convert (val, data->magn_sample_y, data->magn_sensitivity);
                break;
        case SENSOR_CHAN_MAGN_Z:
                lsm6ds3_magn_convert (val, data->magn_sample_z, data->magn_sensitivity);
                break;
        case SENSOR_CHAN_MAGN_XYZ:
                lsm6ds3_magn_convert (val, data->magn_sample_x, data->magn_sensitivity);
                lsm6ds3_magn_convert (val + 1, data->magn_sample_y, data->magn_sensitivity);
                lsm6ds3_magn_convert (val + 2, data->magn_sample_z, data->magn_sensitivity);
                break;
        default:
                return -ENOTSUP;
        }

        return 0;
}

static int lsm6ds3_magn_channel_get (enum sensor_channel chan, struct sensor_value *val, struct lsm6ds3_data *data)
{
        return lsm6ds3_magn_get_channel (chan, val, data);
}
#endif

#if defined(CONFIG_LSM6DS3_EXT0_LPS22HB)
static inline void lps22hb_press_convert (struct sensor_value *val, int32_t raw_val)
{
        /* Pressure sensitivity is 4096 LSB/hPa */
        /* Convert raw_val to val in kPa */
        val->val1 = (raw_val >> 12) / 10;
        val->val2 = (raw_val >> 12) % 10 * 100000 + (((int32_t)((raw_val)&0x0FFF) * 100000L) >> 12);
}

static inline void lps22hb_temp_convert (struct sensor_value *val, int16_t raw_val)
{
        /* Temperature sensitivity is 100 LSB/deg C */
        val->val1 = raw_val / 100;
        val->val2 = (int32_t)raw_val % 100 * (10000);
}
#endif

static int lsm6ds3_channel_get (const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
        struct lsm6ds3_data *data = dev->data;

        switch (chan) {
        case SENSOR_CHAN_ACCEL_X:
        case SENSOR_CHAN_ACCEL_Y:
        case SENSOR_CHAN_ACCEL_Z:
        case SENSOR_CHAN_ACCEL_XYZ:
                lsm6ds3_accel_channel_get (chan, val, data);
                break;
        case SENSOR_CHAN_GYRO_X:
        case SENSOR_CHAN_GYRO_Y:
        case SENSOR_CHAN_GYRO_Z:
        case SENSOR_CHAN_GYRO_XYZ:
                lsm6ds3_gyro_channel_get (chan, val, data);
                break;
#if defined(CONFIG_LSM6DS3_ENABLE_TEMP)
        case SENSOR_CHAN_DIE_TEMP:
                lsm6ds3_gyro_channel_get_temp (val, data);
                break;
#endif
#if defined(CONFIG_LSM6DS3_EXT0_LIS2MDL)
        case SENSOR_CHAN_MAGN_X:
        case SENSOR_CHAN_MAGN_Y:
        case SENSOR_CHAN_MAGN_Z:
        case SENSOR_CHAN_MAGN_XYZ:
                lsm6ds3_magn_channel_get (chan, val, data);
                break;
#endif
#if defined(CONFIG_LSM6DS3_EXT0_LPS22HB)
        case SENSOR_CHAN_PRESS:
                lps22hb_press_convert (val, data->sample_press);
                break;

        case SENSOR_CHAN_AMBIENT_TEMP:
                lps22hb_temp_convert (val, data->sample_temp);
                break;
#endif
        default:
                return -ENOTSUP;
        }

        return 0;
}

static const struct sensor_driver_api lsm6ds3_driver_api = {
        .attr_set = lsm6ds3_attr_set,
#if CONFIG_LSM6DS3_TRIGGER
        .trigger_set = lsm6ds3_trigger_set,
#endif
        .sample_fetch = lsm6ds3_sample_fetch,
        .channel_get = lsm6ds3_channel_get,
};

static int lsm6ds3_init_chip (const struct device *dev)
{
        struct lsm6ds3_data *data = dev->data;
        uint8_t chip_id;

        if (lsm6ds3_reboot (dev) < 0) {
                LOG_DBG ("failed to reboot device");
                return -EIO;
        }

        if (data->hw_tf->read_reg (dev, LSM6DS3_REG_WHO_AM_I, &chip_id) < 0) {
                LOG_DBG ("failed reading chip id");
                return -EIO;
        }
        if (chip_id != LSM6DS3_VAL_WHO_AM_I) {
                LOG_DBG ("invalid chip id 0x%x", chip_id);
                return -EIO;
        }

        LOG_DBG ("chip id 0x%x", chip_id);

        if (lsm6ds3_accel_set_fs_raw (dev, LSM6DS3_DEFAULT_ACCEL_FULLSCALE) < 0) {
                LOG_DBG ("failed to set accelerometer full-scale");
                return -EIO;
        }
        data->accel_sensitivity = LSM6DS3_DEFAULT_ACCEL_SENSITIVITY;

        data->accel_freq = lsm6ds3_odr_to_freq_val (CONFIG_LSM6DS3_ACCEL_ODR);
        if (lsm6ds3_accel_set_odr_raw (dev, CONFIG_LSM6DS3_ACCEL_ODR) < 0) {
                LOG_DBG ("failed to set accelerometer sampling rate");
                return -EIO;
        }

        if (lsm6ds3_gyro_set_fs_raw (dev, LSM6DS3_DEFAULT_GYRO_FULLSCALE) < 0) {
                LOG_DBG ("failed to set gyroscope full-scale");
                return -EIO;
        }
        data->gyro_sensitivity = LSM6DS3_DEFAULT_GYRO_SENSITIVITY;

        data->gyro_freq = lsm6ds3_odr_to_freq_val (CONFIG_LSM6DS3_GYRO_ODR);
        if (lsm6ds3_gyro_set_odr_raw (dev, CONFIG_LSM6DS3_GYRO_ODR) < 0) {
                LOG_DBG ("failed to set gyroscope sampling rate");
                return -EIO;
        }

        if (data->hw_tf->update_reg (dev, LSM6DS3_REG_FIFO_CTRL5, LSM6DS3_MASK_FIFO_CTRL5_FIFO_MODE, 0 << LSM6DS3_SHIFT_FIFO_CTRL5_FIFO_MODE)
            < 0) {
                LOG_DBG ("failed to set FIFO mode");
                return -EIO;
        }

        if (data->hw_tf->update_reg (dev, LSM6DS3_REG_CTRL3_C, LSM6DS3_MASK_CTRL3_C_BDU | LSM6DS3_MASK_CTRL3_C_BLE | LSM6DS3_MASK_CTRL3_C_IF_INC,
                                     (1 << LSM6DS3_SHIFT_CTRL3_C_BDU) | (0 << LSM6DS3_SHIFT_CTRL3_C_BLE) | (1 << LSM6DS3_SHIFT_CTRL3_C_IF_INC))
            < 0) {
                LOG_DBG ("failed to set BDU, BLE and burst");
                return -EIO;
        }

        return 0;
}

static int lsm6ds3_init (const struct device *dev)
{
        const struct lsm6ds3_config *const config = dev->config;
        struct lsm6ds3_data *data = dev->data;

        data->bus = device_get_binding (config->bus_name);
        if (!data->bus) {
                LOG_DBG ("master not found: %s", config->bus_name);
                return -EINVAL;
        }

        config->bus_init (dev);

        if (lsm6ds3_init_chip (dev) < 0) {
                LOG_DBG ("failed to initialize chip");
                return -EIO;
        }

#ifdef CONFIG_LSM6DS3_TRIGGER
        if (lsm6ds3_init_interrupt (dev) < 0) {
                LOG_ERR ("Failed to initialize interrupt.");
                return -EIO;
        }
#endif

#ifdef CONFIG_LSM6DS3_SENSORHUB
        if (lsm6ds3_shub_init_external_chip (dev) < 0) {
                LOG_DBG ("failed to initialize external chip");
                return -EIO;
        }
#endif

        return 0;
}

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "LSM6DS3 driver enabled without any devices"
#endif

/*
 * Device creation macro, shared by LSM6DS3_DEFINE_SPI() and
 * LSM6DS3_DEFINE_I2C().
 */

#define LSM6DS3_DEVICE_INIT(inst)                                                                                                               \
        DEVICE_DT_INST_DEFINE (inst, lsm6ds3_init, NULL, &lsm6ds3_data_##inst, &lsm6ds3_config_##inst, POST_KERNEL,                             \
                               CONFIG_SENSOR_INIT_PRIORITY, &lsm6ds3_driver_api);

/*
 * Instantiation macros used when a device is on a SPI bus.
 */

#define LSM6DS3_HAS_CS(inst) DT_INST_SPI_DEV_HAS_CS_GPIOS (inst)

#define LSM6DS3_DATA_SPI_CS(inst)                                                                                                               \
        {                                                                                                                                       \
                .cs_ctrl = {                                                                                                                    \
                        .gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN (inst),                                                                        \
                        .gpio_dt_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS (inst),                                                                 \
                },                                                                                                                              \
        }

#define LSM6DS3_DATA_SPI(inst) COND_CODE_1 (LSM6DS3_HAS_CS (inst), (LSM6DS3_DATA_SPI_CS (inst)), ({}))

#define LSM6DS3_SPI_CS_PTR(inst) COND_CODE_1 (LSM6DS3_HAS_CS (inst), (&(lsm6ds3_data_##inst.cs_ctrl)), (NULL))

#define LSM6DS3_SPI_CS_LABEL(inst) COND_CODE_1 (LSM6DS3_HAS_CS (inst), (DT_INST_SPI_DEV_CS_GPIOS_LABEL (inst)), (NULL))

#define LSM6DS3_SPI_CFG(inst)                                                                                                                   \
        (&(struct lsm6ds3_spi_cfg) {					\
		.spi_conf = {						\
			.frequency =					\
				DT_INST_PROP(inst, spi_max_frequency),	\
			.operation = (SPI_WORD_SET(8) |			\
				      SPI_OP_MODE_MASTER |		\
				      SPI_MODE_CPOL |			\
				      SPI_MODE_CPHA),			\
			.slave = DT_INST_REG_ADDR(inst),		\
			.cs = LSM6DS3_SPI_CS_PTR(inst),			\
		},							\
		.cs_gpios_label = LSM6DS3_SPI_CS_LABEL(inst),		\
	})

#ifdef CONFIG_LSM6DS3_TRIGGER
#define LSM6DS3_CFG_IRQ(inst)                                                                                                                   \
        .irq_dev_name = DT_INST_GPIO_LABEL (inst, irq_gpios), .irq_pin = DT_INST_GPIO_PIN (inst, irq_gpios),                                    \
        .irq_flags = DT_INST_GPIO_FLAGS (inst, irq_gpios),
#else
#define LSM6DS3_CFG_IRQ(inst)
#endif /* CONFIG_LSM6DS3_TRIGGER */

#define LSM6DS3_CONFIG_SPI(inst)                                                                                                                \
        {                                                                                                                                       \
                .bus_name = DT_INST_BUS_LABEL (inst), .bus_init = lsm6ds3_spi_init, .bus_cfg = {.spi_cfg = LSM6DS3_SPI_CFG (inst)},             \
                COND_CODE_1 (DT_INST_NODE_HAS_PROP (inst, irq_gpios), (LSM6DS3_CFG_IRQ (inst)), ())                                             \
        }

#define LSM6DS3_DEFINE_SPI(inst)                                                                                                                \
        static struct lsm6ds3_data lsm6ds3_data_##inst = LSM6DS3_DATA_SPI (inst);                                                               \
        static const struct lsm6ds3_config lsm6ds3_config_##inst = LSM6DS3_CONFIG_SPI (inst);                                                   \
        LSM6DS3_DEVICE_INIT (inst)

/*
 * Instantiation macros used when a device is on an I2C bus.
 */

#define LSM6DS3_CONFIG_I2C(inst)                                                                                                                \
        {                                                                                                                                       \
                .bus_name = DT_INST_BUS_LABEL(inst),			\
		.bus_init = lsm6ds3_i2c_init,				\
		.bus_cfg = { .i2c_slv_addr = DT_INST_REG_ADDR(inst), },	\
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, irq_gpios),	\
		(LSM6DS3_CFG_IRQ(inst)), ())                                                                                        \
        }

#define LSM6DS3_DEFINE_I2C(inst)                                                                                                                \
        static struct lsm6ds3_data lsm6ds3_data_##inst;                                                                                         \
        static const struct lsm6ds3_config lsm6ds3_config_##inst = LSM6DS3_CONFIG_I2C (inst);                                                   \
        LSM6DS3_DEVICE_INIT (inst)
/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define LSM6DS3_DEFINE(inst) COND_CODE_1 (DT_INST_ON_BUS (inst, spi), (LSM6DS3_DEFINE_SPI (inst)), (LSM6DS3_DEFINE_I2C (inst)))

DT_INST_FOREACH_STATUS_OKAY (LSM6DS3_DEFINE)
