/*
 * Encoder485.h
 *
 *  Created on: Jun 19, 2025
 *      Author: sean
 */

#ifndef SIMPLEFOC_SENSORS_ENCODER485_H_
#define SIMPLEFOC_SENSORS_ENCODER485_H_

#include "ModbusAbsoluteEncoder.h"
#include "main.h"
#include <stdint.h>

class Encoder485 : public ModbusAbsoluteEncoder {
public:
    explicit Encoder485(TIM_HandleTypeDef *htim = nullptr, uint8_t unit_id = 1, UART_HandleTypeDef *huart = nullptr);

    void init() override;

private:
    bool constructQuery(modbus_t* modbusQuary) override;
    UART_HandleTypeDef *huart;
};

#endif /* SIMPLEFOC_SENSORS_ENCODER485_H_ */
