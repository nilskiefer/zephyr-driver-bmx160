#include "bmx160.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bmx160, CONFIG_SENSOR_LOG_LEVEL);

static int bmx160_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct bmx160_data *data = dev->data;
    const struct bmx160_config *cfg = dev->config;

    /* Read acceleration, gyroscope and magnetometer data from sensor */
    uint8_t buf[20];

    /* This driver expects BMI160 + BMM150 data block starting at accel registers. */
    if (i2c_burst_read_dt(&cfg->i2c, 0x12, buf, sizeof(buf)) < 0) {
        return -EIO;
    }

    /* Convert raw values to sensor_value (placeholder conversion) */
    for (int i = 0; i < 3; i++) {
        int16_t raw_a = (int16_t)((buf[i*2+0] << 8) | buf[i*2+1]);
        data->accel[i].val1 = raw_a;
        data->accel[i].val2 = 0;
    }

    for (int i = 0; i < 3; i++) {
        int16_t raw_g = (int16_t)((buf[6 + i*2] << 8) | buf[6 + i*2 + 1]);
        data->gyro[i].val1 = raw_g;
        data->gyro[i].val2 = 0;
    }

    for (int i = 0; i < 3; i++) {
        int16_t raw_m = (int16_t)((buf[12 + i*2] << 8) | buf[12 + i*2 + 1]);
        data->magn[i].val1 = raw_m;
        data->magn[i].val2 = 0;
    }

    return 0;
}

static int bmx160_channel_get(const struct device *dev, enum sensor_channel chan,
                              struct sensor_value *val)
{
    struct bmx160_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_ACCEL_X:
    case SENSOR_CHAN_ACCEL_Y:
    case SENSOR_CHAN_ACCEL_Z:
        *val = data->accel[chan - SENSOR_CHAN_ACCEL_X];
        return 0;
    case SENSOR_CHAN_GYRO_X:
    case SENSOR_CHAN_GYRO_Y:
    case SENSOR_CHAN_GYRO_Z:
        *val = data->gyro[chan - SENSOR_CHAN_GYRO_X];
        return 0;
    case SENSOR_CHAN_MAGN_X:
    case SENSOR_CHAN_MAGN_Y:
    case SENSOR_CHAN_MAGN_Z:
        *val = data->magn[chan - SENSOR_CHAN_MAGN_X];
        return 0;
    default:
        return -ENOTSUP;
    }
}

static int bmx160_init(const struct device *dev)
{
    const struct bmx160_config *cfg = dev->config;

    if (!device_is_ready(cfg->i2c.bus)) {
        return -ENODEV;
    }

    /* Reset sensor */
    uint8_t cmd[2] = {0x7E, 0xB6};
    if (i2c_write_dt(&cfg->i2c, cmd, sizeof(cmd)) < 0) {
        return -EIO;
    }

    k_msleep(100);

    /* Additional configuration would normally be done here */

    return 0;
}

static const struct sensor_driver_api bmx160_api = {
    .sample_fetch = bmx160_sample_fetch,
    .channel_get = bmx160_channel_get,
};

#define BMX160_INST(inst) \
    static struct bmx160_data bmx160_data_##inst; \
    static const struct bmx160_config bmx160_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, bmx160_init, NULL, \
                          &bmx160_data_##inst, &bmx160_config_##inst, \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
                          &bmx160_api);

DT_INST_FOREACH_STATUS_OKAY(BMX160_INST)

