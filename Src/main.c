#include "my_stm_helper.h"
      
#define MPU1_ADDR           0x68
#define REG_PWR_MGMT_1      0x6B
#define REG_ACCEL_XOUT_H    0x3B
#define ACCEL_THRESHOLD     ((int16_t)10000)

volatile bool check_i2c = false;

int main(void) {
    // Enable Peripheral Clocks 
    Clock_Enable(GPIOA_BUS);
    Clock_Enable(GPIOB_BUS);
    Clock_Enable(TIM16_BUS);
    Clock_Enable(TIM3_BUS);
    Clock_Enable(I2C1_BUS);

    // Setup PA0 as LED Output and start off
    GPIO_InitPin(GPIOA, 0, OUTPUT, NONE);
    GPIO_OutputClearPin(GPIOA, 0);

    // Configure PB8 (SCL)/PB9 (SDA) for I2C1
    GPIO_InitPin(GPIOB, 8, ALT, NONE);
    GPIO_SetAltFcn(GPIOB, 8, 6);
    GPIO_InitPin(GPIOB, 9, ALT, NONE);
    GPIO_SetAltFcn(GPIOB, 9, 6);

    // Initialise TIM16 for timeout (1ms ticks i.e. 1 kHz, max count)
    Timer_Init(TIM16, TIM16_IRQn, 47999, 0xFFFF, false);

    // Initialise TIM3 for Main Loop Interrupts (interrupts every 100 ms)
    // PSC = 47999 (1ms ticks), ARR = 99 (100 ticks = 100ms)
    Timer_Init(TIM3, TIM3_IRQn, 47999, 99, true);

    // Initialise I2C1 (100 kHz @ 48 MHz)
    I2C_Master_Init(I2C1,11,19,19);

    // Wake-up the connected IMU (MPU-6050)
    I2C_Master_WriteReg(I2C1, MPU1_ADDR, REG_PWR_MGMT_1, 0x00U); 

    uint8_t mpu1_buf[2];
    while (1) {
        // Use TIM3 to trigger 100ms checking of I2C
        while (!check_i2c);
        check_i2c = false; 
        // Start ACC read at ACC_X high. Auto-increment to read ACC_X low 
        bool mpu1_x_acc = I2C_Master_ReadRegs(I2C1, MPU1_ADDR, REG_ACCEL_XOUT_H, mpu1_buf, 2);

        if (mpu1_x_acc) {
            int16_t accel_x = (int16_t)((mpu1_buf[0] << 8) | mpu1_buf[1]);
            
            if (accel_x > ACCEL_THRESHOLD || accel_x < -ACCEL_THRESHOLD) {
                GPIO_OutputSetPin(GPIOA, 0);       // Turn LED ON as threshold exceeded
            } else {
                GPIO_OutputClearPin(GPIOA, 0);     // LED off as within limits
            }
        } else {
            I2C1->CR1 &= ~(1<<0);
            I2C1->CR1 |=  (1<<0);
        }
    }
}

// TIM3 Interrupt Handler (Executes every 100 ms for main sampling loop)
void TIM3_IRQHandler(void) {
    if (TIM3->SR & (1<<0)) {
        TIM3->SR &= ~(1<<0);    // Clear update interrupt flag     
        check_i2c = true;         
    }
}



