/*!
    This is the C-code for implementing the host MCU ESP32-S3 interfacing  
    with temperature and relative humidity SHT4x integrated circuit, via
    i2c bus.

    @param author Rafael Wagner.

*/
#ifndef I2C_TEMP_SENSOR_H
#define I2C_TEMP_SENSOR_H

#include "cmd_i2ctools.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#define I2C_TEMP_SENSOR_ADD                     0x44
#define I2C_TEMP_SENSOR_CMD_SOFT_RESET          0x94
#define I2C_TEMP_SENSOR_CMD_READ_SERIAL_NUMBER  0x89
#define I2C_TEMP_SENSOR_CMD_READ_DATA           0xFD

#define I2C_TEMP_SENSOR_READ_AFTER_CMD_DELAY_IN_MICROSSEC 10000

#define TASK_TEMP_INIT_STEPS_MS     250
#define TASK_TEMP_READ_INTERVAL_MS  2000


/*!
    Constructs the temp sensor instance.
    \return 0=success 1=fail
*/
int i2c_temp_sensor_construct(void);

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
    FreeRTOS task on CORE0 for creating the instance.
    This means configuring, resetting and initializing 
    the i2c device.
    \param task input argumens
*/
void task_create_temperature_instance(void *arg);

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

/*!
    Returns whether the temperature sensor is 
    operational or not.
    \return 1=is operational 0=is not operatoinal
*/
int i2c_temp_sensor_is_operational(void);

/*!
    Fast CRC-8 implementation using lookup table.
    Polynomial 0x31 (x^8 + x^5 + x^4 + 1)
    Initialization 0xFF
    Reflect input(false) and output(false)
    Final XOR zero.
    \param data array of uint8 bytes
    \param len size of the array in number of uint8 bytes
    \return the calculated CRC-8 value.
*/
uint8_t calculate_cr8x_fast(uint8_t* data, size_t len);

/*!
    Slow CRC-8 implementation by using the polynomial
    bit by bit on the input stream of data.
    Polynomial 0x31 (x^8 + x^5 + x^4 + 1)
    Initialization 0xFF
    Reflect input(false) and output(false)
    Final XOR zero.
    \param data array of uint8 bytes
    \param len size of the array in number of uint8 bytes
    \return the calculated CRC-8 value.
*/
uint8_t calculate_cr8x_slow(uint8_t* data, size_t len);

/*!
    Returns the number of ticks from the processor,
    in microsseconds.
    \return the ticks from the processor.
*/
unsigned long IRAM_ATTR micros();

/*!
    Causes a delay in microsseconds to the ESP32.
*/
void IRAM_ATTR delayMicros(uint32_t us);

#endif