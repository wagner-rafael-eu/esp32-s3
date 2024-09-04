/*!
    This is the C-code for implementing the host MCU ESP32-S3 interfacing  
    with temperature and relative humidity SHT4x integrated circuit, via
    i2c bus.

    @param author Rafael Wagner.

*/
#ifndef I2C_TEMP_SENSOR_H
#define I2C_TEMP_SENSOR_H

#define I2C_TEMP_SENSOR_ADD 0x44
#define I2C_TEMP_SENSOR_CMD_SOFT_RESET 0x94
#define I2C_TEMP_SENSOR_CMD_READ_SERIAL_NUMBER 0x89
#define I2C_TEMP_SENSOR_CMD_READ_DATA 0xFD

#define TASK_TEMP_READ_INTERVAL_MS 2000

/*!
    Configures the temp sensor.
    \return 0=success 1=fail
*/
int i2c_temp_sensor_configure(void);

/*!
    Resets the temp sensor by sending the soft reset command.
    \return 0=success 1=fail
*/
int i2c_temp_sensor_reset(void);

/*!
    Initializes the temp sensor by reading its serial number.
    \return 0=success 1=fail
*/
int i2c_temp_sensor_init(void);

/*!
    Reads temperature and relative humidity raw data, and 
    converts them to human readable values.
    \return 0=success 1=fail
*/
int i2c_temp_sensor_read_data(void);

/*!
    FreeRTOS task on CORE0 for periodically reading values.
    \param task input argumens
*/
void task_retrieve_temperature_data(void *arg);

/*!
    Suspends reading task execution.
    \return 0=success 1=fail
*/
int i2c_temp_sensor_suspend(void);

/*!
    Resumes reading task execution.
    \return 0=success 1=fail
*/
int i2c_temp_sensor_resume(void);

#endif