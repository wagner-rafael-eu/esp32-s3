#include "cmd_i2ctools.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

static uint32_t i2c_frequency = 100 * 1000;
static bool temp_sensor_initialized = false;
i2c_device_config_t i2c_dev_conf;
i2c_master_dev_handle_t dev_handle;

TaskHandle_t temp_sensor_retrieve_data = NULL;  //  Task handler for periodically reading temp sensor data

static const char *TEMP_TAG = "cmd_i2ctools";

int i2c_temp_sensor_configure(void){
    int chip_addr = I2C_TEMP_SENSOR_ADD;

    temp_sensor_initialized = false;
    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = chip_addr,
    };
    i2c_master_dev_handle_t dev_handle;
    if (i2c_master_bus_add_device(tool_bus_handle, &i2c_dev_conf, &dev_handle) != ESP_OK) {
        return 1;
    }

    if (i2c_master_bus_rm_device(dev_handle) != ESP_OK) {
        return 1;
    }
    return 0;

}

int i2c_temp_sensor_reset(void){

    temp_sensor_initialized = false;

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = I2C_TEMP_SENSOR_ADD,
    };

    if (i2c_master_bus_add_device(tool_bus_handle, &i2c_dev_conf, &dev_handle) != ESP_OK) {
        return 1;
    }
    uint8_t cmd = I2C_TEMP_SENSOR_CMD_SOFT_RESET;
    esp_err_t ret = i2c_master_transmit(dev_handle, &cmd, 2, I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TEMP_TAG, "Write OK");
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TEMP_TAG, "Bus is busy");
    } else {
        ESP_LOGW(TEMP_TAG, "Write Failed");
    }

    return 0;

}

int i2c_temp_sensor_init(void){

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = i2c_frequency,
        .device_address = I2C_TEMP_SENSOR_ADD,
    };

    if (i2c_master_bus_add_device(tool_bus_handle, &i2c_dev_conf, &dev_handle) != ESP_OK) {
        return 1;
    }

    int size = 6;
    uint8_t data[size];
    int serial_number = 0;
    int crc = 0;
    uint8_t cmd = I2C_TEMP_SENSOR_CMD_READ_SERIAL_NUMBER;

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &cmd, 1, data, size, I2C_TOOL_TIMEOUT_VALUE_MS);
    serial_number = (data[0]<<24) + (data[1]<<16) + (data[3]<<8) + (data[4]);
    crc = data[2];
    if (ret == ESP_OK) {
            printf("TEMP_SENSOR: IC serial number=%d ", serial_number);
    } else {
            printf("TEMP_SENSOR: error reading IC serial number.");
    }

    if (i2c_master_bus_rm_device(dev_handle) != ESP_OK) {
        return 1;
    }
    temp_sensor_initialized = true;
    xTaskCreatePinnedToCore(
        task_retrieve_temperature_data,     // Function to be performed the task is called
        "task_retrieve_temperature_data",   // Name of the task in text
        3000,                               // Stack size (Memory size assigned to the task)
        NULL,                               // Pointer that will be used as the parameter for the task being created
        5,                                  // Task Priority
        &temp_sensor_retrieve_data,         // The task handler
        0);                                 // xCoreID (Core 0)

    return 0;
}

int i2c_temp_sensor_read_data(void){

    if (i2c_master_bus_add_device(tool_bus_handle, &i2c_dev_conf, &dev_handle) != ESP_OK) {
        return 1;
    }

    int size = 6;
    uint8_t data[size];
    int temperature = 0;
    int relative_humidity = 0;
    int crc = 0;

    uint8_t cmd = I2C_TEMP_SENSOR_CMD_READ_DATA;

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &cmd, 1, data, size, I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) {
            // Temperature: convert raw converted data to degrees Celsius
            temperature = -45 + 175 * ((data[0]<<8)+data[1])/65535;
            crc = data[2];
            if( (temperature >= -40) && (temperature <= 125) ){
                printf("TEMP_SENSOR: temperature %d deg Celsius.\n", temperature);
            }
            else{
                printf("TEMP_SENSOR: error temperature out of IC operational limits %d deg Celsius.\n", temperature);
            }
            // Relative humidity: convert raw converted data to percentage
            relative_humidity = -6 + 125 * ((data[3]<<8)+data[4])/65535;
            crc = data[5];
            if( (relative_humidity >= 0) && (relative_humidity <= 100) ){
                printf("TEMP_SENSOR: relative humidity %d %%.\n", relative_humidity);
            }
            else{
                printf("TEMP_SENSOR: error relative humidity out of IC operational limits %d %%.\n", relative_humidity);
            }
    } else {
            printf("TEMP_SENSOR: error reading temperature and humididy data.");
    }

    if (i2c_master_bus_rm_device(dev_handle) != ESP_OK) {
        return 1;
    }
    return 0;

}

//==============================================================================================//
// Function to periodically read temperature sensor data 
void task_retrieve_temperature_data(void *arg){
  int ret = 1;
  for(;;){
    if( temp_sensor_initialized == true ){
        ret = i2c_temp_sensor_read_data();
        if( ret != 0 ){
            printf("TEMP_SENSOR: IC reading error on FreeRTOS task.\n");
        }
    }
    vTaskDelay(TASK_TEMP_READ_INTERVAL_MS / portTICK_PERIOD_MS);
  }
}

int i2c_temp_sensor_suspend(void){
    temp_sensor_initialized = false;
    return 0;
}

int i2c_temp_sensor_resume(void){
    temp_sensor_initialized = true;
    return 0;
}
