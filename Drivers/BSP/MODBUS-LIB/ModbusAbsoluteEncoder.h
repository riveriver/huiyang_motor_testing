#pragma once

#include "Sensor.h"
#include "main.h"
#include <stdint.h>
#include "Modbus.h"
// modbus多圈绝对值编码器
class ModbusAbsoluteEncoder : public Sensor {
public:
    // Constructor
    explicit ModbusAbsoluteEncoder(TIM_HandleTypeDef *htim = nullptr, uint8_t unit_id = 1);
    virtual ~ModbusAbsoluteEncoder() = default;
    // Initialization
    void init();

    bool isInitialized() const{return encoder_initialized;};

    // Start/stop asynchronous reading driven by timer interrupts
    void startAsyncReading(uint32_t update_frequency_hz = 100);
    void stopAsyncReading();

    // Must be called from the task context each time the timer interrupt notifies
    void asyncUpdateCallback();

    float getAngle();
    float getSensorAngle();

    // Communication helpers
    bool isCommunicationOK() const { return comm_ok; }
    uint32_t getLastUpdateTime() const { return last_update_time; }

    // Frequency statistics helpers
    float getReceiveFrequency() const;
    void resetFrequencyStats();

    static uint16_t rad2reg(float rad);
    static float reg2rad(uint16_t reg);

    static uint16_t u322reg(uint32_t position);
    static uint32_t reg2u32(uint16_t reg);

    void updateAngleFromPosition(uint32_t current_position);

    bool sendModbusQuery();
    bool checkModbusResponse();

    void setAngleUpdateCallback(void (*callback)(uint32_t position));

protected:
    // Internal helpers
    virtual bool constructQuery(modbus_t* modbusQuary) = 0;
    // Modbus constants
    modbusHandler_t ModbusH;
    uint16_t ModbusDATA[8];
    uint8_t unit_id = 1;  // Most devices use 1 by default

private:
    // 用户可设置的角度更新回调函数指针
    void (*angleUpdateCallback)(uint32_t position) = nullptr;
    volatile bool comm_ok;
    enum ModbusState { IDLE, QUERY_SENT } modbus_state;

    // Async state
    volatile float cached_angle;
    volatile bool async_enabled;
    volatile uint32_t last_update_time;
    uint32_t encoder_receive_count;
    uint32_t query_start_time;
    bool first_read;
    // Hardware timer for timer interrupt
    TIM_HandleTypeDef *htim;

    // Multi–turn tracking
    uint32_t last_position;
    int32_t full_rotations;
    uint32_t zero_position;

    static const uint32_t MODBUS_TIMEOUT_MS = 50; // ms

    // Frequency statistics
    uint32_t frequency_start_time;
    uint32_t last_receive_count;

    // encoder init
    bool encoder_initialized;

    // Encoder parameters
    static const uint32_t ENCODER_RESOLUTION = 8192;
    static const float RADIANS_PER_COUNT;
    static const uint32_t ENCODER_ZERO_POSITION = 4195200;

};