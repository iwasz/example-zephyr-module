# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_LSM6DS3            lsm6ds3.c)
zephyr_library_sources_ifdef(CONFIG_LSM6DS3            lsm6ds3_spi.c)
zephyr_library_sources_ifdef(CONFIG_LSM6DS3            lsm6ds3_i2c.c)
zephyr_library_sources_ifdef(CONFIG_LSM6DS3_TRIGGER    lsm6ds3_trigger.c)
zephyr_library_sources_ifdef(CONFIG_LSM6DS3_SENSORHUB  lsm6ds3_shub.c)

