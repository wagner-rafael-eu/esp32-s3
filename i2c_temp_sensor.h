#ifndef I2C_TEMP_SENSOR_H
#define I2C_TEMP_SENSOR_H

#define I2C_TEMP_SENSOR_ADD 0x44
#define I2C_TEMP_SENSOR_CMD_SOFT_RESET 0x94
#define I2C_TEMP_SENSOR_CMD_READ_SERIAL_NUMBER 0x89
#define I2C_TEMP_SENSOR_CMD_READ_DATA 0xFD

#define TASK_TEMP_READ_INTERVAL_MS 2000

int i2c_temp_sensor_configure(void);
int i2c_temp_sensor_reset(void);
int i2c_temp_sensor_init(void);
int i2c_temp_sensor_suspend(void);
int i2c_temp_sensor_resume(void);
int i2c_temp_sensor_read_data(void);
void task_retrieve_temperature_data(void *arg);

#endif