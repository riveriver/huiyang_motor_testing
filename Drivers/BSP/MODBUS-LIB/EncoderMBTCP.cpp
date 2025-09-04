#include "EncoderMBTCP.h"

#if ENABLE_TCP == 1
#include <cstring>
#include <errno.h>
#include <sys/time.h>
#include "cmsis_os.h"
// Include lwIP headers only in the implementation
extern "C" {
    #include "lwip/sockets.h"
    #include "lwip/inet.h"
}

EncoderMBTCP::EncoderMBTCP(TIM_HandleTypeDef *htim, uint8_t unit_id, const char* encoder_ip, uint16_t encoder_port)
  : ModbusAbsoluteEncoder(htim, unit_id),
    encoder_ip(encoder_ip),
    encoder_port(encoder_port)
{
}

void EncoderMBTCP::init() {
    // Attempt to open socket but do not block forever
    ModbusH.uModbusType = MB_MASTER;
    //ModbusH.uModbusType = MB_MASTER;
    //ModbusH.port =  &huart3; //this is not used for TCP
    // ModbusH.u8id = 1; //slave ID
    ModbusH.u8id = 0; //slave ID for master always 0
    ModbusH.u16timeOut = 5;
    ModbusH.EN_Port = NULL; // No RS485
    //ModbusH2.EN_Port = LD2_GPIO_Port; // RS485 Enable
    //ModbusH2.EN_Pin = LD2_Pin; // RS485 Enable
    ModbusH.u16regs = ModbusDATA;
    ModbusH.u16regsize= sizeof(ModbusDATA)/sizeof(ModbusDATA[0]);
    ModbusH.xTypeHW = TCP_HW; // TCP hardware


    ModbusAbsoluteEncoder::init();
}

bool EncoderMBTCP::constructQuery(modbus_t* modbusQuary) {
    modbusQuary->u8id = unit_id;                      // 从站地址03
    modbusQuary->u8fct = MB_FC_READ_REGISTERS;  // 功能码03：读取保持寄存器
    modbusQuary->u16RegAdd = 0x0000;            // 寄存器起始地址0x00
    modbusQuary->u16CoilsNo = 2;                // 读取2个寄存器（0x00和0x01）
    modbusQuary->u16reg = ModbusDATA;           // 数据存储缓冲区
    ip4addr_aton(encoder_ip, (ip4_addr_t *)&modbusQuary->xIpAddress);
    modbusQuary->u16Port = encoder_port;
    modbusQuary->u8clientID = 0;
    return true;
}
#endif /* ENABLE_TCP */