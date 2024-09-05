#include "i2c_temp_sensor.h"
#include "esp_timer.h"

static uint32_t i2c_frequency = 100 * 1000;
static bool temp_sensor_initialized = false;
i2c_device_config_t i2c_dev_conf;
i2c_master_dev_handle_t dev_handle;

//  Task handler for temp sensor instance creation
TaskHandle_t temp_sensor_create_instance = NULL;    
//  Task handler for periodically reading temp sensor data
TaskHandle_t temp_sensor_retrieve_data = NULL;      

static const char *TEMP_TAG = "cmd_i2ctools";
static int i2c_temp_sensor_constructed = 0;

int i2c_temp_sensor_construct(void){
    i2c_temp_sensor_constructed = 0;

    xTaskCreatePinnedToCore(
        task_create_temperature_instance,   // Function to be performed the task is called
        "task_create_temperature_instance", // Name of the task in text
        2048,                               // Stack size (Memory size assigned to the task)
        NULL,                               // Pointer that will be used as the parameter for the task being created
        4,                                  // Task Priority
        &temp_sensor_create_instance,       // The task handler
        0);                                 // xCoreID (Core 0)

    return 0;
}

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
    uint8_t cmd = I2C_TEMP_SENSOR_CMD_READ_SERIAL_NUMBER;

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &cmd, 1, data, size, I2C_TOOL_TIMEOUT_VALUE_MS);
    serial_number = (data[0]<<24) + (data[1]<<16) + (data[3]<<8) + (data[4]);

    // CRC-8 handling
    uint8_t crc = 0;
    crc = data[2];
    uint8_t crc_calc = 0;
    crc_calc = calculate_cr8x_fast(data, 2);
    if( crc != crc_calc){
            printf("TEMP_SENSOR: CRC-8 error. Received=%x Calculated=%x.", crc, crc_calc);
    }
    crc = data[5];
    crc_calc = calculate_cr8x_fast(data+3*sizeof(uint8_t), 2);
    if( crc != crc_calc){
            printf("TEMP_SENSOR: CRC-8 error. Received=%x Calculated=%x.", crc, crc_calc);
    }
    // 

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
        2048,                               // Stack size (Memory size assigned to the task)
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
    uint8_t crc = 0;

    uint8_t cmd = I2C_TEMP_SENSOR_CMD_READ_DATA;

    esp_err_t ret = i2c_master_transmit(dev_handle, &cmd, 1, I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TEMP_TAG, "Write OK");
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TEMP_TAG, "Bus is busy");
    } else {
        ESP_LOGW(TEMP_TAG, "Write Failed");
    }
 
    // Delaying 10ms to receive temp and humidity data, 
    // just as mentioned in the datasheet of the component SHT4x.
    delayMicros( I2C_TEMP_SENSOR_READ_AFTER_CMD_DELAY_IN_MICROSSEC );
    
    ret = i2c_master_receive(dev_handle, data, size, I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK) {
            // Temperature: convert raw converted data to degrees Celsius
            temperature = -45 + 175 * ((data[0]<<8)+data[1])/65535;
            // CRC-8 handling
            crc = data[2];
            uint8_t crc_calc = 0;
            crc_calc = calculate_cr8x_fast(data, 2);
            if( crc != crc_calc){
                    printf("TEMP_SENSOR: CRC-8 error. Received=%x Calculated=%x.", crc, crc_calc);
            }
            // 
            if( (temperature >= -40) && (temperature <= 125) ){
                printf("TEMP_SENSOR: temperature %d deg Celsius.\n", temperature);
            }
            else{
                printf("TEMP_SENSOR: error temperature out of IC operational limits %d deg Celsius.\n", temperature);
            }
            // Relative humidity: convert raw converted data to percentage
            relative_humidity = -6 + 125 * ((data[3]<<8)+data[4])/65535;
            // CRC-8 handling
            crc = data[5];
            crc_calc = 0;
            crc_calc = calculate_cr8x_fast(data+3*sizeof(uint8_t), 2);
            if( crc != crc_calc){
                    printf("TEMP_SENSOR: CRC-8 error. Received=%x Calculated=%x.", crc, crc_calc);
            }
            // 
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
// Task for creating the temperature sensor instance
void task_create_temperature_instance(void *arg){
    int ret = 1;
    ret = i2c_temp_sensor_configure();
    if( ret != 0 ){
        ESP_LOGE(TEMP_TAG, "Failed to configure the temp sensor=%d", ret);
    }
    else{
        vTaskDelay(TASK_TEMP_INIT_STEPS_MS / portTICK_PERIOD_MS);
        ret = i2c_temp_sensor_reset();
        if( ret != 0 ){
            ESP_LOGE(TEMP_TAG, "Failed to reset the temp sensor=%d", ret);
        }
        else{
            vTaskDelay(TASK_TEMP_INIT_STEPS_MS / portTICK_PERIOD_MS);
            ret = i2c_temp_sensor_init();
            if( ret != 0 ){
                ESP_LOGE(TEMP_TAG, "Failed to initialize the temp sensor=%d", ret);
            }
            else{
                vTaskDelay(TASK_TEMP_INIT_STEPS_MS / portTICK_PERIOD_MS);
                i2c_temp_sensor_constructed = 1;
            }
        }
    }
}

//==============================================================================================//
// Task to periodically read temperature sensor data 
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

int i2c_temp_sensor_is_operational(void){
    return temp_sensor_initialized?1:0;
}

const uint8_t crc8x_table[256] = {
0x00,0x31,0x62,0x53,0xC4,0xF5,0xA6,0x97,0xB9,0x88,0xDB,0xEA,0x7D,0x4C,0x1F,0x2E,
0x43,0x72,0x21,0x10,0x87,0xB6,0xE5,0xD4,0xFA,0xCB,0x98,0xA9,0x3E,0x0F,0x5C,0x6D,
0x86,0xB7,0xE4,0xD5,0x42,0x73,0x20,0x11,0x3F,0x0E,0x5D,0x6C,0xFB,0xCA,0x99,0xA8,
0xC5,0xF4,0xA7,0x96,0x01,0x30,0x63,0x52,0x7C,0x4D,0x1E,0x2F,0xB8,0x89,0xDA,0xEB,
0x3D,0x0C,0x5F,0x6E,0xF9,0xC8,0x9B,0xAA,0x84,0xB5,0xE6,0xD7,0x40,0x71,0x22,0x13,
0x7E,0x4F,0x1C,0x2D,0xBA,0x8B,0xD8,0xE9,0xC7,0xF6,0xA5,0x94,0x03,0x32,0x61,0x50,
0xBB,0x8A,0xD9,0xE8,0x7F,0x4E,0x1D,0x2C,0x02,0x33,0x60,0x51,0xC6,0xF7,0xA4,0x95,
0xF8,0xC9,0x9A,0xAB,0x3C,0x0D,0x5E,0x6F,0x41,0x70,0x23,0x12,0x85,0xB4,0xE7,0xD6,
0x7A,0x4B,0x18,0x29,0xBE,0x8F,0xDC,0xED,0xC3,0xF2,0xA1,0x90,0x07,0x36,0x65,0x54,
0x39,0x08,0x5B,0x6A,0xFD,0xCC,0x9F,0xAE,0x80,0xB1,0xE2,0xD3,0x44,0x75,0x26,0x17,
0xFC,0xCD,0x9E,0xAF,0x38,0x09,0x5A,0x6B,0x45,0x74,0x27,0x16,0x81,0xB0,0xE3,0xD2,
0xBF,0x8E,0xDD,0xEC,0x7B,0x4A,0x19,0x28,0x06,0x37,0x64,0x55,0xC2,0xF3,0xA0,0x91,
0x47,0x76,0x25,0x14,0x83,0xB2,0xE1,0xD0,0xFE,0xCF,0x9C,0xAD,0x3A,0x0B,0x58,0x69,
0x04,0x35,0x66,0x57,0xC0,0xF1,0xA2,0x93,0xBD,0x8C,0xDF,0xEE,0x79,0x48,0x1B,0x2A,
0xC1,0xF0,0xA3,0x92,0x05,0x34,0x67,0x56,0x78,0x49,0x1A,0x2B,0xBC,0x8D,0xDE,0xEF,
0x82,0xB3,0xE0,0xD1,0x46,0x77,0x24,0x15,0x3B,0x0A,0x59,0x68,0xFF,0xCE,0x9D,0xAC };

uint8_t calculate_cr8x_fast(uint8_t* data, size_t len) {
    uint8_t crc = 0xFF; // init value
    for (size_t i = 0; i < len; i++) {
         crc = crc8x_table[data[i] ^ crc];
     }
   return crc;
}

uint8_t calculate_cr8x_slow(uint8_t* data, size_t len)
{
   uint8_t crc = 0xFF; // init value
   size_t i, j;
   for (i = 0; i < len; i++) {
      crc ^= data[i];
      for (j = 0; j < 8; j++) {
          if ((crc & 0x80) != 0)
              crc = (uint8_t)((crc << 1) ^ 0x31);
          else
              crc <<= 1;
      }
  }
  return crc;
 }

unsigned long IRAM_ATTR micros()
{
    return (unsigned long)(esp_timer_get_time());
}

void IRAM_ATTR delayMicros(uint32_t us)
{
    uint32_t m = micros();
    if (us)
    {
        uint32_t e = (m + us);
        if (m > e)
        { //overflow
            while (micros() > e)
            {
                //NOP();
                asm("nop");
            }
        }
        while (micros() < e)
        {
            //NOP();
            asm("nop");
        }
    }
}