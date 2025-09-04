/*
 * Encoder485.cpp
 *
 *  Created on: Jun 19, 2025
 *      Author: sean
 */


#include "Encoder485.h"
#include <cmath>


Encoder485::Encoder485(TIM_HandleTypeDef *htim, uint8_t unit_id, UART_HandleTypeDef *huart)
  : ModbusAbsoluteEncoder(htim, unit_id),
    huart(huart)
{
}

void Encoder485::init(){
    /* Modbus Master initialization */
  ModbusH.uModbusType = MB_MASTER;
  ModbusH.port = huart; 
  ModbusH.u8id = 0; //master ID
  ModbusH.u16timeOut = 1000;
  ModbusH.EN_Port = NULL; // No RS485
  ModbusH.u16regs = ModbusDATA;
  ModbusH.u16regsize= sizeof(ModbusDATA)/sizeof(ModbusDATA[0]);
  ModbusH.xTypeHW = USART_HW;
  
  ModbusAbsoluteEncoder::init();
}

bool Encoder485::constructQuery(modbus_t* modbusQuary) {
    modbusQuary->u8id = unit_id;                      // 从站地址03
    modbusQuary->u8fct = MB_FC_READ_REGISTERS;  // 功能码03：读取保持寄存器
    modbusQuary->u16RegAdd = 0x0000;            // 寄存器起始地址0x00
    modbusQuary->u16CoilsNo = 2;                // 读取2个寄存器（0x00和0x01）
    modbusQuary->u16reg = ModbusDATA;           // 数据存储缓冲区
    return true;
}