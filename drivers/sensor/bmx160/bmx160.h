#ifndef ZEPHYR_DRIVERS_SENSOR_BMX160_BMX160_H_
#define ZEPHYR_DRIVERS_SENSOR_BMX160_BMX160_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

struct bmx160_config {
    struct i2c_dt_spec i2c;
};

struct bmx160_data {
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    struct sensor_value magn[3];
};

#endif /* ZEPHYR_DRIVERS_SENSOR_BMX160_BMX160_H_ */
