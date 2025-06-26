/* Bosch BMX160 driver internal API
 *
 * Copyright (c) 2023 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BMX160_BMX160_H_
#define ZEPHYR_DRIVERS_SENSOR_BMX160_BMX160_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT bosch_bmx160

/* Common register addresses */
#define BMX160_REG_CHIPID     0x00
#define BMX160_REG_ACC_CONF   0x40
#define BMX160_REG_ACC_RANGE  0x41
#define BMX160_REG_GYR_CONF   0x42
#define BMX160_REG_GYR_RANGE  0x43
#define BMX160_REG_DATA_START 0x12
#define BMX160_REG_CMD        0x7E
#define BMX160_SPI_START      0x7F

#define BMX160_CMD_SOFT_RESET 0xB6
#define BMX160_CHIP_ID        0xD8

#define BMX160_ACC_CONF_ODR_MASK 0x0F
#define BMX160_GYR_CONF_ODR_MASK 0x0F

/* Range register values */
#define BMX160_ACC_RANGE_2G          0x3
#define BMX160_ACC_RANGE_4G          0x5
#define BMX160_ACC_RANGE_8G          0x8
#define BMX160_ACC_RANGE_16G         0xC

#define BMX160_GYR_RANGE_2000DPS     0x0
#define BMX160_GYR_RANGE_1000DPS     0x1
#define BMX160_GYR_RANGE_500DPS      0x2
#define BMX160_GYR_RANGE_250DPS      0x3
#define BMX160_GYR_RANGE_125DPS      0x4

/* Default configuration values derived from Kconfig */
#if defined(CONFIG_BMX160_ACCEL_RANGE_4G)
#define BMX160_DEFAULT_RANGE_ACC  BMX160_ACC_RANGE_4G
#elif defined(CONFIG_BMX160_ACCEL_RANGE_8G)
#define BMX160_DEFAULT_RANGE_ACC  BMX160_ACC_RANGE_8G
#elif defined(CONFIG_BMX160_ACCEL_RANGE_16G)
#define BMX160_DEFAULT_RANGE_ACC  BMX160_ACC_RANGE_16G
#else
#define BMX160_DEFAULT_RANGE_ACC  BMX160_ACC_RANGE_2G
#endif

#if defined(CONFIG_BMX160_GYRO_RANGE_1000DPS)
#define BMX160_DEFAULT_RANGE_GYR  BMX160_GYR_RANGE_1000DPS
#elif defined(CONFIG_BMX160_GYRO_RANGE_500DPS)
#define BMX160_DEFAULT_RANGE_GYR  BMX160_GYR_RANGE_500DPS
#elif defined(CONFIG_BMX160_GYRO_RANGE_250DPS)
#define BMX160_DEFAULT_RANGE_GYR  BMX160_GYR_RANGE_250DPS
#elif defined(CONFIG_BMX160_GYRO_RANGE_125DPS)
#define BMX160_DEFAULT_RANGE_GYR  BMX160_GYR_RANGE_125DPS
#else
#define BMX160_DEFAULT_RANGE_GYR  BMX160_GYR_RANGE_2000DPS
#endif

#if defined(CONFIG_BMX160_ACCEL_ODR_50)
#define BMX160_DEFAULT_ODR_ACC   7
#elif defined(CONFIG_BMX160_ACCEL_ODR_200)
#define BMX160_DEFAULT_ODR_ACC   9
#elif defined(CONFIG_BMX160_ACCEL_ODR_400)
#define BMX160_DEFAULT_ODR_ACC   10
#elif defined(CONFIG_BMX160_ACCEL_ODR_800)
#define BMX160_DEFAULT_ODR_ACC   11
#else
#define BMX160_DEFAULT_ODR_ACC   8
#endif

#if defined(CONFIG_BMX160_GYRO_ODR_50)
#define BMX160_DEFAULT_ODR_GYR   7
#elif defined(CONFIG_BMX160_GYRO_ODR_200)
#define BMX160_DEFAULT_ODR_GYR   9
#elif defined(CONFIG_BMX160_GYRO_ODR_400)
#define BMX160_DEFAULT_ODR_GYR   10
#elif defined(CONFIG_BMX160_GYRO_ODR_800)
#define BMX160_DEFAULT_ODR_GYR   11
#else
#define BMX160_DEFAULT_ODR_GYR   8
#endif

struct bmx160_config {
    struct i2c_dt_spec i2c;
};

struct bmx160_data {
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    struct sensor_value magn[3];
};

#endif /* ZEPHYR_DRIVERS_SENSOR_BMX160_BMX160_H_ */
