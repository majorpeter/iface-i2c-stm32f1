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

#include <assert.h>

Stm32F1I2c::Stm32F1I2c(const InitStruct* initStruct) {
    i2c = initStruct->i2cPeriph;

    assert(rccEnable(i2c, initStruct->sdaSclPort) == 0);

    I2C_InitTypeDef I2C_InitStruct;
    I2C_InitStruct.I2C_ClockSpeed = initStruct->clockSpeedHz;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(i2c, &I2C_InitStruct);
    I2C_Cmd(i2c, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = initStruct->sdaPin | initStruct->sclPin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(initStruct->sdaSclPort, &GPIO_InitStruct);
}

int8_t Stm32F1I2c::rccEnable(const I2C_TypeDef* i2c, const GPIO_TypeDef* sdaSclPort) {
    if (i2c == I2C1) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    } else if (i2c == I2C2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    } else {
        return -1;
    }

    if (sdaSclPort == GPIOA) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    } else if (sdaSclPort == GPIOB) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    } else if (sdaSclPort == GPIOC) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    } else if (sdaSclPort == GPIOD) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    } else if (sdaSclPort == GPIOE) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    } else if (sdaSclPort == GPIOF) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    } else {
        return -2;
    }

    return 0;
}

void Stm32F1I2c::sendStart() {
    while (I2C_GetFlagStatus(i2c, I2C_FLAG_BUSY));
    I2C_GenerateSTART(i2c, ENABLE);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT));
}

void Stm32F1I2c::sendStop() {
    I2C_GenerateSTOP(i2c, ENABLE);
    while (I2C_GetFlagStatus(i2c, I2C_FLAG_STOPF));
}

int8_t Stm32F1I2c::sendSlaveAddress(uint8_t address, uint8_t direction) {
    I2C_Send7bitAddress(i2c, address, direction);

    uint32_t i2c_event = 0;
    if (direction == I2C_Direction_Transmitter) {
        i2c_event = I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED;
    } else if (direction == I2C_Direction_Receiver) {
        i2c_event = I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED;
    }

    uint32_t timeout = 100000;
    while (!I2C_CheckEvent(i2c, i2c_event)) {
        timeout--;
        if (timeout == 0) {
            return Nack;
        }
    }
    return Ok;
}

void Stm32F1I2c::sendByte(uint8_t byte) {
    I2C_SendData(i2c, byte);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t Stm32F1I2c::receiveByte(bool ack) {
    I2C_AcknowledgeConfig(i2c, ack ? ENABLE : DISABLE);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_RECEIVED));

    return I2C_ReceiveData(i2c);
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
