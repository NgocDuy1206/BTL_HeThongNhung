//#include <stm32f10x.h>
//#include <stdint.h>

//#define RCC_APB2ENR   (*(volatile uint32_t *)0x40021018)
//#define GPIOA_CRL     (*(volatile uint32_t *)0x40010800)
//#define GPIOA_ODR     (*(volatile uint32_t *)0x4001080C)
//#define GPIOB_CRL     (*(volatile uint32_t *)0x40010C00)
//#define GPIOB_ODR     (*(volatile uint32_t *)0x40010C0C)
//#define GPIOC_CRH     (*(volatile uint32_t *)0x40011004)
//#define GPIOC_ODR     (*(volatile uint32_t *)0x4001100C)

//// Macros for setting/clearing bits
//#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
//#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

//#define LCD_RS_PIN  (1U << 0)  // PB0
//#define LCD_RW_PIN  (1U << 1)  // PB1
//#define LCD_E_PIN   (1U << 13)  // PC13
//#define LCD_DATA_PINS_MASK  0xFF // PA0-PA7

//void LCD_SendCommand(unsigned char cmd);
//void LCD_SendData(unsigned char data);
//void LCD_SendString(char* str);
//void GPIO_Config(void);
//void LCD_Init(void);
//void Delay_ms(unsigned int time);

//int main(void) {
//    GPIO_Config(); // Configure GPIO for LCD
//    LCD_Init();    // Initialize the LCD

//    
//    while (1) {
//        // Main loop
//    }
//}

//void GPIO_Config(void) {
//    // Enable clock for GPIOA and GPIOB
//    SET_BIT(RCC_APB2ENR, (1U << 2)); // GPIOA clock
//    SET_BIT(RCC_APB2ENR, (1U << 3)); // GPIOB clock
//		SET_BIT(RCC_APB2ENR, (1U << 4)); // GPIOC clock

//    // Configure PA0-PA7 as output push-pull
//    GPIOA_CRL = 0x11111111;

//    // Configure PB0-PB2 as output push-pull
//    GPIOB_CRL &= ~(0xFF);
//    GPIOB_CRL |= 0x11;
//	
//		GPIOC_CRH = (1 << 20);
//}

//void LCD_SendCommand(uint8_t command) {
//    CLEAR_BIT(GPIOB_ODR, LCD_RS_PIN); // RS = 0 for command
//    CLEAR_BIT(GPIOB_ODR, LCD_RW_PIN); // RW = 0 for write

//     // Put command on data bus
//		
//		SET_BIT(GPIOC_ODR, LCD_E_PIN);	// E = 1 for enable
//	
//		GPIOA_ODR = command;
//		
//    CLEAR_BIT(GPIOC_ODR, LCD_E_PIN);  // E = 0 for disable
//}

//void LCD_SendData(uint8_t data) {
//    SET_BIT(GPIOB_ODR, LCD_RS_PIN);   // RS = 1 for data
//    CLEAR_BIT(GPIOB_ODR, LCD_RW_PIN); // RW = 0 for write
//    

//     // Put data on data bus
//		
//		SET_BIT(GPIOC_ODR, LCD_E_PIN);    // E = 1 for enable
//	
//		GPIOA_ODR = data;
//		
//    CLEAR_BIT(GPIOC_ODR, LCD_E_PIN);  // E = 0 for disable
//		
//}

//void LCD_Init(void) 
//{
//		LCD_SendCommand(0x38); 	
//		LCD_SendCommand(0x0C); 
//		LCD_SendCommand(0x01); 
//		LCD_SendString("HELLO"); 
//		LCD_SendCommand(0xC0); 
//		LCD_SendString("BYE"); 
//}

//void LCD_SendString(char* str) {
//   for (unsigned int i = 0; str[i] != 0; i++)
//	{
//			LCD_SendData(str[i]);
//	}
//}

//void Delay_ms(unsigned int time)
//{
//		unsigned int t = time * 12000;
//		while(t > 0) {t--;}
//}


#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>

#define MPU6050_ADDRESS 0xD0  // Ð?a ch? I2C c?a MPU6050

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40


#define FALL_THRESHOLD_LOW 0.5  // Ngu?ng th?p cho t?ng gia t?c (g)
#define FALL_THRESHOLD_HIGH 2.0 // Ngu?ng cao cho t?ng gia t?c (g)
#define FALL_TIME_THRESHOLD 200 // Ngu?ng th?i gian cho cú ngã (ms)


volatile float Threshold = (1.58*9.81); 
volatile int16_t X, Y, Z;
enum status{off, on};
enum status system = on;

void SysClkConf_72MHz(void);
void Led_Init();

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(uint8_t reg, uint8_t data);
void I2C_Write_Addr(uint8_t addr);
void I2C_Write_Addr_Reg(uint8_t addr);
uint8_t I2C_Read_Data(uint8_t addr);

void MPU6050_Init(void);
void MPU6050_Read_Accel(int16_t *x, int16_t *y, int16_t *z);

float Calculate_Magnitude(int16_t x, int16_t y, int16_t z);
int Detect_Fall(int16_t x, int16_t y, int16_t z);
void Convert_Unit(int16_t *x, int16_t *y, int16_t *z);

void EXTI_Config(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);



int main(void) {
		SysClkConf_72MHz();
    I2C_Init();
    MPU6050_Init();
		Led_Init();
		int16_t x, y, z;
    while (1) {
				MPU6050_Read_Accel(&x, &y, &z);
				float g = Calculate_Magnitude(x, y, z);
    }
}
void Led_Init(void)
{
		RCC->APB2ENR |= (1<<4);
		GPIOC->CRH |= 3 << 20;
		GPIOC->ODR &= ~(1 << 13);
}
void I2C_Init(void) {
    // Kh?i t?o các chân I2C và thi?t l?p t?c d? I2C
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;  // B?t clock cho Port B
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;  // B?t clock cho I2C1

    // C?u hình PB6 và PB7 là chân I2C1 SCL và SDA
    GPIOB->CRL |= GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7;
    
    // Thi?t l?p I2C1
    I2C1->CR2 = 36;  // Clock APB1 = 36MHz
    I2C1->CCR = 180; // T?c d? I2C = 100kHz
    I2C1->TRISE = 37; // Tang t?c t?i da cho I2C
    I2C1->CR1 |= I2C_CR1_PE; // Kích ho?t I2C
}

void I2C_Start(void) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));  // Ð?i bit SB du?c set
}

void I2C_Stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}
void I2C_Write_Addr(uint8_t addr)
{
		I2C1->DR = addr;
		while (!(I2C1->SR1 & I2C_SR1_ADDR)); 
		(void)I2C1->SR2;
}
void I2C_Write_Addr_Reg(uint8_t addr)
{
		I2C1->DR = addr;
		while (!(I2C1->SR1 & I2C_SR1_TXE));
}
void I2C_Write_Data(uint8_t data) {
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}

void I2C_Write(uint8_t reg, uint8_t data)
{
		I2C_Start();
		I2C_Write_Addr(MPU6050_ADDRESS);
		I2C_Write_Addr_Reg(reg);
		I2C_Write_Data(data);
		I2C_Stop();
}
uint8_t I2C_Read_Data(uint8_t addr)
{
		uint8_t data;
		I2C_Start();
		I2C_Write_Addr(MPU6050_ADDRESS);
		I2C_Write_Addr_Reg(addr);
		I2C_Start();
		I2C_Write_Addr(MPU6050_ADDRESS | 0x01);
		I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_STOP;
    while (!(I2C1->SR1 & I2C_SR1_RXNE)); // cho tin hieu interrupt
    data = I2C1->DR;
		return data;
		
}

void MPU6050_Init(void) {
    I2C_Write(MPU6050_PWR_MGMT_1, 0x00); // wake up MPU6050
    I2C_Write(MPU6050_CONFIG, 0x00); // tat FSYNC, dai 260Hz
    I2C_Write(MPU6050_ACCEL_CONFIG, 0x00); // dai cua cam bien gia toc ?2g
    I2C_Write(MPU6050_INT_ENABLE, 0x01); // bat ngat thong bao khi du lieu san sang
    I2C_Write(MPU6050_INT_PIN_CFG, 0x10); // xoa cac bit int status khi doc
}

void MPU6050_Read_Accel(int16_t *x, int16_t *y, int16_t *z) {
    
     *x = ((int16_t)I2C_Read_Data(MPU6050_ACCEL_XOUT_H) << 8) | I2C_Read_Data(MPU6050_ACCEL_XOUT_L);
     *y = ((int16_t)I2C_Read_Data(MPU6050_ACCEL_YOUT_H) << 8) | I2C_Read_Data(MPU6050_ACCEL_YOUT_L);
     *z = ((int16_t)I2C_Read_Data(MPU6050_ACCEL_ZOUT_H) << 8) | I2C_Read_Data(MPU6050_ACCEL_ZOUT_L);
    
}

void SysClkConf_72MHz(void) {
    //su dung hse
    RCC->CR |= RCC_CR_HSEON;
    while((RCC->CR & RCC_CR_HSERDY) == 0); // doi san sang

    //cau hinh PLL 
    RCC->CFGR |= RCC_CFGR_PLLMULL9; // *9 = systemclock = 72MHz
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6; // ADC prescale 6.
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; //APB1 prescale 2.

    //chon bo nguon hse cho pll
    RCC->CFGR |= RCC_CFGR_PLLSRC; // PLLSRC HSE

    //bat pll
    RCC->CR |= RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == 0); // wait PLLRDY.

    //doi tu sysclock sang pll
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) == 0); //wait SWS.

    //tat nguon he thong (hsi)
    RCC->CR &= ~(RCC_CR_HSION); // off HSION
    while((RCC->CR & RCC_CR_HSIRDY) == RCC_CR_HSIRDY);
}



float Calculate_Magnitude(int16_t x, int16_t y, int16_t z)
{
		float a = (x / 16384.0f) * 9.81;
		float b = (y / 16384.0f) * 9.81;
		float c = (z / 16384.0f) * 9.81;
		return sqrt(a*a + b*b + c*c);
}

// hàm phát hien ngã 1 - true, 0 - false
int Detect_Fall(int16_t x, int16_t y, int16_t z) {
    static unsigned long fall_start_time = 0;
    float magnitude = Calculate_Magnitude(x, y, z);

//    if (magnitude < FALL_THRESHOLD_LOW) {
//        if (fall_start_time == 0) {
//            fall_start_time = millis();
//        }
//        if (millis() - fall_start_time > FALL_TIME_THRESHOLD) {
//            fall_start_time = 0;
//            return 1;
//        }
//    } else if (magnitude > FALL_THRESHOLD_HIGH) {
//        fall_start_time = 0;
//    }
		if (magnitude > Threshold) return 1;
    return 0;
}

void EXTI_Config(void) {
    // cap xung cho afio v? gpioa de su dung lam ngat cong tac
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
		
    // pull up cho interrupt tu mpu6050 
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    GPIOA->CRL |= GPIO_CRL_CNF0_1;
    GPIOA->ODR |= 1<<0;

    // pullup cho cong tac tat mo he thong
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOA->CRL |= GPIO_CRL_CNF1_1;
		GPIOA->ODR |= 1<<1;
		
    // cau h?nh afio v? exti
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA | AFIO_EXTICR1_EXTI1_PA; // bat 2 chan pa0 va pa1 voi nhiem vu ngat ngoai
    EXTI->PR |= EXTI_PR_PR0 | EXTI_PR_PR1; // xoa pending 
    
		//chon suon xuong
		EXTI->FTSR |= EXTI_FTSR_TR0 | EXTI_FTSR_TR1;

    // xoa suon len    
    EXTI->RTSR &= ~(EXTI_RTSR_TR0);
    EXTI->RTSR &= ~(EXTI_RTSR_TR1);

    // chon interrupt xoa event
    EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1;
    EXTI->EMR &= ~(EXTI_EMR_MR0);
    EXTI->EMR &= ~(EXTI_EMR_MR1);

    // dat muc do uu tien
		// ngat cua mpu6050
    NVIC_SetPriority(EXTI0_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_IRQn);
		//ngat cua nut tat bat he thong
    NVIC_SetPriority(EXTI1_IRQn, 0);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

void EXTI1_IRQHandler(void) {
    if(EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR |= EXTI_PR_PR1;
        system = !system; // dao nguoc che do
    }
}

void EXTI0_IRQHandler(void) {
		if(EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR |= EXTI_PR_PR0;
        if (system) {
            MPU6050_Read_Accel(&X, &Y, &Z);
            if (Detect_Fall(X, Y, Z)) {
//								lcd_Writedata(1);
								GPIOC->ODR ^= (1 << 13);
//								delayMs(100);
						} else {
//								lcd_Writedata(0);
                GPIOC->BSRR |= (1 << 13); // tat den
						}
				}
		}
}
