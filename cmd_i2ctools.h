/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifndef CMD_I2CTOOLS_H
#define CMD_I2CTOOLS_H

#include "driver/i2c_master.h"
#include "i2c_temp_sensor.h"

#define I2C_TOOL_TIMEOUT_VALUE_MS (50)

#ifdef __cplusplus
extern "C" {
#endif

void register_i2ctools(void);

extern i2c_master_bus_handle_t tool_bus_handle;

#ifdef __cplusplus
}
#endif

#endif
