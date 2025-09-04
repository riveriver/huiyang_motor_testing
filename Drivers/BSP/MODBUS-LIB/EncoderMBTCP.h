#ifndef SIMPLEFOC_SENSORS_ENCODERMBTCP_H_
#define SIMPLEFOC_SENSORS_ENCODERMBTCP_H_

#include "ModbusAbsoluteEncoder.h"

#if ENABLE_TCP == 1
#include "main.h"
#include <stdint.h>
#include <cmath>

class EncoderMBTCP : public ModbusAbsoluteEncoder {
public:
    // Constructor
    explicit EncoderMBTCP(TIM_HandleTypeDef *htim = nullptr, uint8_t unit_id = 1, const char* encoder_ip = "192.168.1.34", uint16_t encoder_port = 502);

    // Initialization
    void init() override;



private:

    bool constructQuery(modbus_t* modbusQuary) override;
    // Modbus TCP constants
    const char* encoder_ip;
    uint16_t encoder_port;
};

#endif /* ENABLE_TCP */
#endif /* SIMPLEFOC_SENSORS_ENCODERMBTCP_H_ */ 
