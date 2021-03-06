/*
 * Stm32F1I2c.h
 *
 *  Created on: 2018. máj. 18.
 *      Author: peti
 */

#ifndef MODULES_IFACE_I2C_STM32F1_STM32F1I2C_H_
#define MODULES_IFACE_I2C_STM32F1_STM32F1I2C_H_

#include <iface-i2c/I2cInterface.h>
#include <stm32f10x.h>

class Stm32F1I2c: public I2cInterface {
public:
    enum Error {Ok = 0, Nack = -1};
    typedef struct {
        I2C_TypeDef* i2cPeriph;
        GPIO_TypeDef* sdaSclPort;
        uint16_t sdaPin;
        uint16_t sclPin;
        uint32_t clockSpeedHz;
    } InitStruct;

    Stm32F1I2c(const InitStruct* initStruct);
    virtual ~Stm32F1I2c() {}

    virtual int8_t write(uint8_t slave_address, const uint8_t* data, uint16_t length);
    virtual int8_t read(uint8_t slave_address, uint8_t* data, uint16_t length);
private:
    I2C_TypeDef* i2c;

    int8_t rccEnable(const I2C_TypeDef* i2c, const GPIO_TypeDef* sdaSclPort);
    void sendStart();
    void sendStop();
    int8_t sendSlaveAddress(uint8_t address, uint8_t direction);
    void sendByte(uint8_t byte);
    uint8_t receiveByte(bool ack);
};

#endif /* MODULES_IFACE_I2C_STM32F1_STM32F1I2C_H_ */
