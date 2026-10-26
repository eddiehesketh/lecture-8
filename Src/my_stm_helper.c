#include "my_stm_helper.h"

void Clock_Enable(PeripheralBus_t peripheral) {
    switch (peripheral) {
        case GPIOA_BUS:  RCC->IOPENR  |= (1 << 0);  break;
        case GPIOB_BUS:  RCC->IOPENR  |= (1 << 1);  break;
        case TIM3_BUS:   RCC->APBENR1 |= (1 << 1);  break;
        case TIM16_BUS:  RCC->APBENR2 |= (1 << 17); break;
        case TIM17_BUS:  RCC->APBENR2 |= (1 << 18); break;
        case I2C1_BUS:   RCC->APBENR1 |= (1 << 21); break;
    }
}

void GPIO_InitPin(GPIO_TypeDef *port, uint8_t pin, GPIOMode_t mode, GPIOPull_t pull) {
    port->MODER &= ~(0b11 << (pin * 2));
    port->MODER |=  (mode << (pin * 2));

    port->PUPDR &= ~(0b11 << (pin * 2));
    port->PUPDR |=  (pull << (pin * 2));
}

void Timer_Init(TIM_TypeDef *timer, IRQn_Type irq_type, uint16_t psc, uint16_t arr, bool enable_interrupt) {
    timer->PSC = psc;
    timer->ARR = arr;

    if (enable_interrupt) {
        timer->DIER |= 1<<0;            
        NVIC_EnableIRQ(irq_type);       
    }
    timer->EGR |= (1 << 0);
    timer->SR &= ~(1 << 0);
    timer->CR1 |= 1<<0;                 
}

void GPIO_OutputClearPin(GPIO_TypeDef *port, uint8_t pin) {
    port->ODR &= ~(1 << pin);
}

void GPIO_OutputSetPin(GPIO_TypeDef *port, uint8_t pin) {
    port->ODR |= (1 << pin);
}

void GPIO_OutputTogglePin(GPIO_TypeDef *port, uint8_t pin) {
    port->ODR ^= (1 << pin);
}

bool GPIO_InputReadPin(GPIO_TypeDef *port, uint8_t pin) {
    return (port->IDR & (1 << pin)) != 0;       
}

void GPIO_SetAltFcn(GPIO_TypeDef *port, uint8_t pin, uint8_t num_af) {
    uint8_t idx = pin / 8;                      
    uint8_t shift = (pin % 8) * 4;              

    port->AFR[idx] &= ~(0xF << shift);
    port->AFR[idx] |=  (num_af << shift);
}

void TIM3_PWM1_Init(uint8_t ch, uint32_t duty_us) {
    switch (ch) {
        case 1:
            TIM3->CCMR1 &= ~(0b11<<0 | 0b111 << 4);                 
            TIM3->CCMR1 |= (0b110 << 4) | (1<<3);                   
            TIM3->CCR1   = duty_us;                                 
            TIM3->CCER  |= (1<<0);                                  
            break;
        case 2:
            TIM3->CCMR1 &= ~(0b11<<8 | 0b111 << 12);                 
            TIM3->CCMR1 |= (0b110 << 12) | (1<<11);                   
            TIM3->CCR2   = duty_us;                                 
            TIM3->CCER  |= (1<<4);                                  
            break;
        case 3:
            TIM3->CCMR2 &= ~(0b11<<0 | 0b111 << 4);                 
            TIM3->CCMR2 |= (0b110 << 4) | (1<<3);                   
            TIM3->CCR3   = duty_us;                                 
            TIM3->CCER  |= (1<<8);                                  
            break;
        case 4:
            TIM3->CCMR2 &= ~(0b11<<8 | 0b111 << 12);                 
            TIM3->CCMR2 |= (0b110 << 12) | (1<<11);                   
            TIM3->CCR4   = duty_us;                                 
            TIM3->CCER  |= (1<<12);                                 
            break;
    }
    TIM3->EGR |= (1<<0);                                            
    TIM3->CR1 |= (1<<0);                                            
}

void TIM3_PWM_SetPulse(uint8_t ch, uint16_t duty_us) {
    switch (ch) {
        case 1: TIM3->CCR1 = duty_us; break;
        case 2: TIM3->CCR2 = duty_us; break;
        case 3: TIM3->CCR3 = duty_us; break;
        case 4: TIM3->CCR4 = duty_us; break;
    }
}

void EXTI_InitPin(EXTIPort_t port, uint8_t pin, EXTITrigger_t trigger, IRQn_Type irq_type) {
    uint8_t idx = pin / 4;              
    uint8_t shift = (pin % 4) * 8;      
    
    EXTI->EXTICR[idx] &= ~(0xFF << shift);
    EXTI->EXTICR[idx] |=  (port << shift);
    
    EXTI->FTSR1 &= ~(1 << pin);
    EXTI->RTSR1 &= ~(1 << pin);
    
    if (trigger == FALLING || trigger == BOTH) EXTI->FTSR1 |= (1 << pin);      
    if (trigger == RISING || trigger == BOTH)   EXTI->RTSR1 |= (1 << pin);      

    EXTI->IMR1 |= (1 << pin);
    EXTI->FPR1 |= (1 << pin);
    EXTI->RPR1 |= (1 << pin);

    NVIC_EnableIRQ(irq_type);
}

// --- I2C Helper Implementations ---

bool check_timeout(uint16_t start_tick, uint16_t timeout_ms) {
    uint16_t current_tick = (uint16_t)TIM16->CNT;
    return (uint16_t)(current_tick - start_tick) >= timeout_ms;
}

void I2C_Master_Init(I2C_TypeDef *i2c, uint8_t presc, uint8_t sclh, uint8_t scll) {
    // Disable I2C peripheral before modifying timing
    i2c->CR1 &= ~(1<<0);
    // Configure the TIMINGR register
    i2c->TIMINGR = ((uint32_t)presc << 28) | ((uint32_t)sclh << 8) | ((uint32_t)scll<<0);
    // Re-enable the I2C peripheral
    i2c->CR1 |= (1<<0);
}

bool I2C_Master_WriteReg(I2C_TypeDef *i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    uint16_t start_tick;
    const uint16_t timeout_ms = 10;                             

    // Wait for i2c bus free
    start_tick = (uint16_t)TIM16->CNT;                          // use current counter value as start time (1 tick = 1ms)
    while (i2c->ISR & (1<<15)) {
        // Check Busy flag. If timeout > 10 ms; bus is busy. Exit
        if (check_timeout(start_tick, timeout_ms)) return false;
    }

    // Write the target device register address over I2C () and one byte of data
    // device_addrs, num_bytes, autoend = true, start transmission
    i2c->CR2 &= ~(1<<10);                               // Configure to write
    i2c->CR2 = (dev_addr << 1) | (2 << 16) | (1<<25) | (1<<13);

    start_tick = (uint16_t)TIM16->CNT;                      // reset start_tick counter
    // Wait for communication of device address to complete
    while (!(i2c->ISR & (1<<1))) {
        // Check status of TXIS
        if (check_timeout(start_tick, timeout_ms)) return false;
    }
    // Device address has been written, TXDR now empty
    i2c->TXDR = reg_addr;                               // Address of register on peripheral to read

    // Wait for communication of register address to complete
    while (!(i2c->ISR & (1<<1))) {
        // Check status of TXIS
        if (check_timeout(start_tick, timeout_ms)) return false;
    }
    i2c->TXDR = data;                                      // Actual data to write to peripheral

    start_tick = (uint16_t)TIM16->CNT;                  // Reset Start Tick counter
    while (!(i2c->ISR & (1<<5))) {
        // Wait for communication with peripheral to be complete (check stop flag)
        if (check_timeout(start_tick, timeout_ms)) return false;
    }
    i2c->ICR |= (1<<5);                 // Write 1 to clear stop flag

    return true;
}

bool I2C_Master_ReadRegs(I2C_TypeDef *i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *rx_buf, uint16_t size) {
    uint16_t start_tick;
    const uint16_t timeout_ms = 10; 

    // Wait for i2c bus free
    start_tick = (uint16_t)TIM16->CNT;                          // use current counter value as start time (1 tick = 1ms)
    while (i2c->ISR & (1<<15)) {
        // Check Busy flag. If timeout > 10 ms; bus is busy. Exit
        if (check_timeout(start_tick, timeout_ms)) return false;
    }

    // Write the target device register address over I2C ()
    // device_addrs, num)bytes, autoend = true, start transmission
    i2c->CR2 &= ~(1<<10);                               // Configure to write
    i2c->CR2 = (dev_addr << 1) | (1 << 16) | (1<<25) | (1<<13);

    start_tick = (uint16_t)TIM16->CNT;                      // reset start_tick counter
    // Wait for communication of device address to complete
    while (!(i2c->ISR & (1<<1))) {
        // Check status of TXIS
        if (check_timeout(start_tick, timeout_ms)) return false;
    }

    // Device address has been written, TXDR now empty
    i2c->TXDR = reg_addr;                               // Address of register on peripheral to read

    start_tick = (uint16_t)TIM16->CNT;                  // Reset Start Tick counter
    while (!(i2c->ISR & (1<<5))) {
        // Wait for communication with peripheral to be complete
        if (check_timeout(start_tick, timeout_ms)) return false;
    }
    i2c->ICR |= (1<<5);                 // Write 1 to clear stop flag

    // Small pause to check state of I2C line
    start_tick = (uint16_t)TIM16->CNT;                  // Reset Start Tick counter
    while (i2c->ISR & (1<<15)) {
        // Check Busy flag. If timeout > 10 ms; bus is busy. Exit
        if (check_timeout(start_tick, timeout_ms)) return false;
    }

    // Read back requested bytes from attached peripheral
    // device_addrs, num_bytes, set to read, autoend = true, start transmission
    i2c->CR2 = (dev_addr << 1) | ((uint32_t)size << 16) | (1<<10) | (1<<25) | (1<<13);

    // Populate buffer with number of bytes that are expected to be received from peripheral to micro
    for (uint16_t i = 0; i < size; i++) {
        start_tick = (uint16_t)TIM16->CNT;                              // Reset Start tick counter
        while (!(i2c->ISR & (1<<2))) {
            // While no data has been received, check if a NACK transmitted across or timeout
            if (i2c->ISR & (1<<4)) return false;
            if (check_timeout(start_tick, timeout_ms)) return false;
        }
        // Store received data in software buffer
        rx_buf[i] = (uint8_t)i2c->RXDR;
    }

    start_tick = (uint16_t)TIM16->CNT;                  // Reset Start Tick counter
    while (!(i2c->ISR & (1<<5))) {
        // Wait for communication with peripheral to be complete
        if (check_timeout(start_tick, timeout_ms)) return false;
    }
    i2c->ICR |= (1<<5);                 // Write 1 to clear stop flag

    return true;
}
