/*
 * Stm32F1I2c.cpp
 *
 *  Created on: 2018. máj. 18.
 *      Author: peti
 */

#include "Stm32F1I2c.h"
#include <stm32f10x.h>
#include <stm32f10x_i2c.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

#define I2Cx_RCC        RCC_APB1Periph_I2C1
#define I2Cx            I2C1
#define I2C_GPIO_RCC    RCC_APB2Periph_GPIOB
#define I2C_GPIO        GPIOB
#define I2C_PIN_SDA     GPIO_Pin_7
#define I2C_PIN_SCL     GPIO_Pin_6

Stm32F1I2c::Stm32F1I2c() {
    this->init();
}

void Stm32F1I2c::init() {
    RCC_APB1PeriphClockCmd(I2Cx_RCC, ENABLE);
    I2C_InitTypeDef I2C_InitStruct;
    I2C_InitStruct.I2C_ClockSpeed = 100000;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2Cx, &I2C_InitStruct);
    I2C_Cmd(I2Cx, ENABLE);

    RCC_APB2PeriphClockCmd(I2C_GPIO_RCC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = I2C_PIN_SCL | I2C_PIN_SDA;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(I2C_GPIO, &GPIO_InitStruct);
}

void Stm32F1I2c::sendStart() {
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2Cx, ENABLE);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
}

void Stm32F1I2c::sendStop() {
    I2C_GenerateSTOP(I2Cx, ENABLE);
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
}

int8_t Stm32F1I2c::sendSlaveAddress(uint8_t address, uint8_t direction) {
    I2C_Send7bitAddress(I2Cx, address, direction);

    uint32_t i2c_event = 0;
    if (direction == I2C_Direction_Transmitter) {
        i2c_event = I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED;
    } else if (direction == I2C_Direction_Receiver) {
        i2c_event = I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED;
    }

    uint32_t timeout = 100000;
    while (!I2C_CheckEvent(I2Cx, i2c_event)) {
        timeout--;
        if (timeout == 0) {
            return Nack;
        }
    }
    return Ok;
}

void Stm32F1I2c::sendByte(uint8_t byte) {
    I2C_SendData(I2Cx, byte);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t Stm32F1I2c::receiveByte(bool ack) {
    I2C_AcknowledgeConfig(I2Cx, ack ? ENABLE : DISABLE);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));

    return I2C_ReceiveData(I2Cx);
}

int8_t Stm32F1I2c::write(uint8_t slave_address, const uint8_t* data, uint16_t length) {
    sendStart();
    int8_t result = sendSlaveAddress(slave_address, I2C_Direction_Transmitter);
    if (result == Ok) {
        for (uint16_t i = 0; i < length; i++) {
            sendByte(data[i]);
        }
    }
    sendStop();
    return 0;
}

int8_t Stm32F1I2c::read(uint8_t slave_address, uint8_t* data, uint16_t length) {
    sendStart();
    int8_t result = sendSlaveAddress(slave_address, I2C_Direction_Receiver);
    if (result == Ok) {
        for (uint16_t i = 0; i < length; i++) {
            data[i] = receiveByte(i != length - 1);
        }
    }
    sendStop();
    return result;
}
