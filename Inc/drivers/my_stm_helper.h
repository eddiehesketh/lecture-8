#ifndef DRIVERS_H
#define DRIVERS_H

#include "stm32c0xx.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GPIOA_BUS,
    GPIOB_BUS,
    TIM3_BUS,
    TIM16_BUS,
    TIM17_BUS,
    I2C1_BUS
} PeripheralBus_t;

typedef enum {
    INPUT  = 0b00,
    OUTPUT = 0b01,
    ALT    = 0b10,
    ANALOG = 0b11
} GPIOMode_t;

typedef enum {
    NONE      = 0b00,
    PULL_UP   = 0b01,
    PULL_DOWN = 0b10
} GPIOPull_t;

typedef enum {
    FALLING,
    RISING,
    BOTH
} EXTITrigger_t;

typedef enum { 
    EXTI_GPIOA = 0x00, 
    EXTI_GPIOB = 0x01
} EXTIPort_t;

void Clock_Enable(PeripheralBus_t peripheral);
void GPIO_InitPin(GPIO_TypeDef *port, uint8_t pin, GPIOMode_t mode, GPIOPull_t pull);
void Timer_Init(TIM_TypeDef *timer, IRQn_Type irq_type, uint16_t psc, uint16_t arr, bool enable_interrupt);
void GPIO_OutputTogglePin(GPIO_TypeDef *port, uint8_t pin);
void GPIO_OutputSetPin(GPIO_TypeDef *port, uint8_t pin);
void GPIO_OutputClearPin(GPIO_TypeDef *port, uint8_t pin);
bool GPIO_InputReadPin(GPIO_TypeDef *port, uint8_t pin);
void GPIO_SetAltFcn(GPIO_TypeDef *port, uint8_t pin, uint8_t num_af);
void TIM3_PWM1_Init(uint8_t ch, uint32_t duty_us);
void TIM3_PWM_SetPulse(uint8_t ch, uint16_t duty_us);
void EXTI_InitPin(EXTIPort_t port, uint8_t pin, EXTITrigger_t trigger, IRQn_Type irq_type);

// I2C Wrappers
//void I2C_Master_Init(I2C_TypeDef *i2c, uint32_t timingr);
bool I2C_Master_WriteReg(I2C_TypeDef *i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
bool I2C_Master_ReadRegs(I2C_TypeDef *i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *rx_buf, uint16_t size);
void I2C_Master_Init(I2C_TypeDef *i2c, uint8_t presc, uint8_t sclh, uint8_t scll);
bool check_timeout(uint16_t start_tick, uint16_t timeout_ms);
#endif // DRIVERS_H