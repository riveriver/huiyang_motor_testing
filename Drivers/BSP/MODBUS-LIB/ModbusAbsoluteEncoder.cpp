#include "ModbusAbsoluteEncoder.h"
#include <cstdio>
#include "retarget.h"
// Define static constant
const float ModbusAbsoluteEncoder::RADIANS_PER_COUNT = 2.0f * M_PI / ModbusAbsoluteEncoder::ENCODER_RESOLUTION;

ModbusAbsoluteEncoder::ModbusAbsoluteEncoder(TIM_HandleTypeDef *htim, uint8_t unit_id)
  : unit_id(unit_id),
    comm_ok(false),
    modbus_state(IDLE),
    cached_angle(0.0f),
    async_enabled(false),
    last_update_time(0),
    encoder_receive_count(0),
    query_start_time(0),
    first_read(true),
    htim(htim),
    last_position(0),
    full_rotations(0),
    zero_position(0),
    frequency_start_time(0),
    last_receive_count(0),
    encoder_initialized(false)
{
}

void ModbusAbsoluteEncoder::init() {
    // Initialise timer stats
    //Initialize Modbus library
    ModbusInit(&ModbusH);
    //Start capturing traffic on serial Port
    ModbusStart(&ModbusH);
    frequency_start_time = HAL_GetTick();
    last_receive_count = 0;
    encoder_initialized = true;
}

bool ModbusAbsoluteEncoder::sendModbusQuery() {
    modbus_t telegram;
    constructQuery(&telegram);
    // 发送Modbus查询（非阻塞）
    ModbusDATA[0] = ~ModbusDATA[0];
    ModbusQuery(&ModbusH, telegram);

    modbus_state = QUERY_SENT;
    query_start_time = HAL_GetTick();
    return true;
}

bool ModbusAbsoluteEncoder::checkModbusResponse() {
    // 检查Modbus响应是否准备好
    // 这里需要根据具体的Modbus库实现来检查
    // 假设ModbusQueryV2是阻塞版本，我们需要非阻塞检查
    
    // 简化实现：检查Modbus状态
    uint32_t u32NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes or timeouts
    if(u32NotificationValue != OP_OK_QUERY)
    {
        return false;
    } else {
        // 成功接收到响应
        encoder_receive_count++;
        uint32_t current_position = ((uint32_t)ModbusDATA[0] << 16) | ModbusDATA[1];
        updateAngleFromPosition(current_position);
        comm_ok = true;
        last_update_time = HAL_GetTick();
        // 通讯完成，检查结果
        return true;
    }
}

void ModbusAbsoluteEncoder::startAsyncReading(uint32_t update_frequency_hz) {
    if(!htim) {
        async_enabled = false;
        return;
    }

    // Configure TIM ARR based on requested frequency (assuming clock already set as in Encoder485)
    uint32_t arr_value = (100000 / update_frequency_hz) - 1;
    if(arr_value > 65535) arr_value = 65535;
    if(arr_value < 10) arr_value = 10;
    __HAL_TIM_DISABLE(htim);
    htim->Instance->ARR = arr_value;
    __HAL_TIM_SET_COUNTER(htim, 0);
    HAL_TIM_Base_Start_IT(htim);

    async_enabled = true;
    modbus_state = IDLE;

    // Initial query so first value is available quickly
    sendModbusQuery();
}

void ModbusAbsoluteEncoder::stopAsyncReading() {
    async_enabled = false;
    modbus_state = IDLE;
    if(htim) {
        HAL_TIM_Base_Stop_IT(htim);
    }
}


float ModbusAbsoluteEncoder::getAngle() {
    if(async_enabled) {
        return cached_angle;
    }
    // 构建Modbus查询结构体
    modbus_t telegram;
    constructQuery(&telegram);
    // 发送Modbus查询（非阻塞）
    ModbusDATA[0] = ~ModbusDATA[0];
    
    uint32_t result = ModbusQueryV2(&ModbusH, telegram);
    
    if (result != OP_OK_QUERY) {
        if (first_read) {
            return 0.0f;
        } else {
            return (float)(last_position % ENCODER_RESOLUTION) * RADIANS_PER_COUNT + 
                    (float)full_rotations * 2.0f * M_PI;
        }
    }
    
    uint32_t current_position = ((uint32_t)ModbusDATA[0] << 16) | ModbusDATA[1];
    updateAngleFromPosition(current_position);
    
    return cached_angle;
}

void ModbusAbsoluteEncoder::asyncUpdateCallback() {
    if(!async_enabled) return;

    switch(modbus_state) {
        case IDLE:
            sendModbusQuery();
//             printf("Sending query...");
            break;
        case QUERY_SENT:
            if(checkModbusResponse()) {
                modbus_state = IDLE;
                // printf("Received response...");
            } else {
                // timeout
                comm_ok = false;
                modbus_state = IDLE;
                printf("Timeout, retrying...");
            }
            break;
    }
}

float ModbusAbsoluteEncoder::getSensorAngle() {
    return (float)(last_position % ENCODER_RESOLUTION) * RADIANS_PER_COUNT;
}

float ModbusAbsoluteEncoder::getReceiveFrequency() const {
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - frequency_start_time;
    if(elapsed < 100) return 0.0f;
    uint32_t diff = encoder_receive_count - last_receive_count;
    return (float)diff * 1000.0f / (float)elapsed;
}

void ModbusAbsoluteEncoder::resetFrequencyStats() {
    frequency_start_time = HAL_GetTick();
    last_receive_count = encoder_receive_count;
}

uint16_t ModbusAbsoluteEncoder::rad2reg(float rad) {
    return (uint16_t)(((uint32_t)(rad * ((float)ENCODER_RESOLUTION / (2.0f * M_PI))) - ENCODER_ZERO_POSITION));
}

float ModbusAbsoluteEncoder::reg2rad(uint16_t reg) {
    return (float)(((uint32_t)reg + ENCODER_ZERO_POSITION) * (2.0f * M_PI) / (float)ENCODER_RESOLUTION);
}

uint16_t ModbusAbsoluteEncoder::u322reg(uint32_t position) {
    return (uint16_t)(position - ENCODER_ZERO_POSITION);
}

uint32_t ModbusAbsoluteEncoder::reg2u32(uint16_t reg) {
    return (uint32_t)reg + ENCODER_ZERO_POSITION;
}

void ModbusAbsoluteEncoder::updateAngleFromPosition(uint32_t current_position) {
    if(first_read) {
        zero_position = current_position;
        first_read = false;
    }

    last_position = current_position;
    cached_angle = current_position * RADIANS_PER_COUNT;
    if(angleUpdateCallback) {
        angleUpdateCallback(current_position);
    }
}

void ModbusAbsoluteEncoder::setAngleUpdateCallback(void (*callback)(uint32_t position)) {
    angleUpdateCallback = callback;
}
