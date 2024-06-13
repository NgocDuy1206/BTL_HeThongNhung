


#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>

// dia chi cua mpu 6050
#define MPU6050_ADDRESS 0xD0  

// dia chi cac thanh ghi can dung cua mpu6050
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
#define FALL_TIME_THRESHOLD 200d // Ngu?ng th?i gian cho cú ngã (ms)

float Sensitivity = 16384.0f;		// Do nhay thiet bi
float G = 9.81;									// gia toc trai dat

volatile int16_t X, Y, Z;
enum status{off, on};
enum status system = on;

// function câu hình chung
void SysClkConf_72MHz(void);
void Delay_Ms(uint8_t time);

// các function xu lý led
void Led_Init(void);
void LCD_Init(void);
void LCD_Send_Command(unsigned char addr);
void LCD_Send_Data(unsigned char data);
void GPIO_Init_Pin(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void LCD_Enable(void);
void LCD_Send_String(char* str);
void LCD_Mode_4bit(unsigned char addr);
void LCD_Clear(void);

// các function dùng I2C
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(uint8_t reg, uint8_t data);
void I2C_Write_Addr(uint8_t addr);
void I2C_Write_Addr_Reg(uint8_t addr);
void I2C_Write_Data(uint8_t data);
uint8_t I2C_Read_Data(uint8_t addr);

void MPU6050_Init(void);
void MPU6050_Read_Accel(int16_t *x, int16_t *y, int16_t *z);

// các function tính toán
float Calculate_Magnitude(int16_t x, int16_t y, int16_t z);
int Detect_Fall(int16_t x, int16_t y, int16_t z);
void Convert_Unit(int16_t *x, int16_t *y, int16_t *z);

// các function dành cho interrupt
void EXTI_Config(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);

void Initial(void);
void Run(void);

int main(void) {
		Initial();
		Run();
}

// khoi tao toan bo cac thanh phan
void Initial(void)
{
		SysClkConf_72MHz();
    I2C_Init();
		LCD_Init();
    MPU6050_Init();
		EXTI_Config();
}


void Run(void)
{
		int16_t x, y, z;
    while (1) {
				if (system == on)
				{
						MPU6050_Read_Accel(&x, &y, &z);
						if (Detect_Fall == 1){
								LCD_Clear();
								LCD_Send_String("FALL");
								GPIOC->ODR &= ~(1<<13);					
						} else {
								GPIOC->ODR &= ~(1<<13);	
						}
							
				} else {
						LCD_Clear();
						LCD_Send_String("OVER");
				}
    }
}


// cau hinh cho giao thuc I2C
void I2C_Init(void) {
    // Bat I2C và GPIOB
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;  // Bat clock cho Port B
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;  // Bat clock cho I2C1

    // cau hinh cho 2 chan pb6 (du lieu) va pb7 (clock) 
		// output 
    GPIOB->CRL |= GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7;
    
    // Thi?t l?p I2C1
    I2C1->CR2 = 36;  // Clock APB1 = 36MHz
    I2C1->CCR = 180; // T?c d? I2C = 100kHz
    I2C1->TRISE = 37; // Tang t?c t?i da cho I2C
    I2C1->CR1 |= I2C_CR1_PE; // Kích ho?t I2C
}

// m? giao th?c I2C
void I2C_Start(void) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));  // Ð?i bit SB du?c set
}

// dóng giao th?c I2C
void I2C_Stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}

// gui dia chi cua slave
void I2C_Write_Addr(uint8_t addr)
{
		I2C1->DR = addr;
		while (!(I2C1->SR1 & I2C_SR1_ADDR)); 
		(void)I2C1->SR2;
}

// gui dia chi thanh ghi muon doc
void I2C_Write_Addr_Reg(uint8_t addr)
{
		I2C1->DR = addr;
		while (!(I2C1->SR1 & I2C_SR1_TXE));
}

// gui du lieu
void I2C_Write_Data(uint8_t data) {
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}


// gui du lieu den thanh ghi mong muon
void I2C_Write(uint8_t reg, uint8_t data)
{
		I2C_Start();
		I2C_Write_Addr(MPU6050_ADDRESS);
		I2C_Write_Addr_Reg(reg);
		I2C_Write_Data(data);
		I2C_Stop();
}

// doc du lieu cua thanh ghi
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


// khoi tao mpu6050
void MPU6050_Init(void) {
    I2C_Write(MPU6050_PWR_MGMT_1, 0x00); // wake up MPU6050
    I2C_Write(MPU6050_CONFIG, 0x00); // tat FSYNC, dai 260Hz
    I2C_Write(MPU6050_ACCEL_CONFIG, 0x00); // dai cua cam bien gia toc ?2g
    I2C_Write(MPU6050_INT_ENABLE, 0x01); // bat ngat thong bao khi du lieu san sang
    I2C_Write(MPU6050_INT_PIN_CFG, 0x10); // xoa cac bit int status khi doc
}


// doc gia tri thu duoc tu cam bien mpu6050
void MPU6050_Read_Accel(int16_t *x, int16_t *y, int16_t *z) {
    
     *x = ((int16_t)I2C_Read_Data(MPU6050_ACCEL_XOUT_H) << 8) | I2C_Read_Data(MPU6050_ACCEL_XOUT_L);
     *y = ((int16_t)I2C_Read_Data(MPU6050_ACCEL_YOUT_H) << 8) | I2C_Read_Data(MPU6050_ACCEL_YOUT_L);
     *z = ((int16_t)I2C_Read_Data(MPU6050_ACCEL_ZOUT_H) << 8) | I2C_Read_Data(MPU6050_ACCEL_ZOUT_L);
    
}


// cau hinh chung cua he thong
void SysClkConf_72MHz(void) {
    //su dung hse
    RCC->CR |= RCC_CR_HSEON;
    while((RCC->CR & RCC_CR_HSERDY) == 0); 			// doi san sang

    //cau hinh PLL 
    RCC->CFGR |= RCC_CFGR_PLLMULL9; 						// *9 = systemclock = 72MHz
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6; 					// ADC prescale 6.
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; 					//APB1 prescale 2.

    //chon bo nguon hse cho pll
    RCC->CFGR |= RCC_CFGR_PLLSRC; 							// PLLSRC HSE

    //bat pll
    RCC->CR |= RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == 0); 			// wait PLLRDY.

    //doi tu sysclock sang pll
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) == 0); 		//wait SWS.

    //tat nguon he thong (hsi)
    RCC->CR &= ~(RCC_CR_HSION); 								// off HSION
    while((RCC->CR & RCC_CR_HSIRDY) == RCC_CR_HSIRDY);
}


// delay chuong trinh mili giay
void Delay_Ms(uint8_t time)
{
	for(int i = 0; i < time; i++){
			SysTick->LOAD = 9000-1;
			SysTick->VAL = 0;
			SysTick->CTRL |= SysTick_CTRL_ENABLE;
			while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG));
			SysTick->CTRL &= ~SysTick_CTRL_ENABLE;
	}
}
// tinh do lon gia toc
float Calculate_Magnitude(int16_t x, int16_t y, int16_t z)
{		
		float a = (x / Sensitivity) * G;
		float b = (y / Sensitivity) * G;
		float c = (z / Sensitivity) * G;
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
		if (magnitude > FALL_THRESHOLD_HIGH * G) return 1;
    return 0;
}


// cau hinh cho interrupt o chan pa0 và pa1
void EXTI_Config(void) {
    // cap xung cho afio va gpioa 
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
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PA; // line 0 va 1 cua chan pa
    EXTI->PR |= EXTI_PR_PR1; 														// xoa pending 
    
		//chon suon xuong
		EXTI->FTSR |= EXTI_FTSR_TR1;

    // xoa suon len    
    EXTI->RTSR &= ~(EXTI_RTSR_TR1);

    // chon interrupt xoa event
    EXTI->IMR |= EXTI_IMR_MR1;
    EXTI->EMR &= ~(EXTI_EMR_MR1);

    // dat muc do uu tien
		//ngat cua nut tat bat he thong
    NVIC_SetPriority(EXTI1_IRQn, 0);
    NVIC_EnableIRQ(EXTI1_IRQn);
}


// xu ly ngat pa1
void EXTI1_IRQHandler(void) {
    if(EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR |= EXTI_PR_PR1;
        system = !system; 
    }
}

// khoi tao LCD
void LCD_Init(void)
{
		GPIO_Init_Pin();
		Delay_ms(1);
		LCD_Mode_4bit(0x33);
		LCD_Mode_4bit(0x32);
		
		LCD_Send_Command(0x28);
		LCD_Send_Command(0x02);
		LCD_Send_Command(0x0C);
		LCD_Send_Command(0x01);
	
		LCD_Send_String("WELCOM");
}	
// vao che do 4 bit cua LCD
void LCD_Mode_4bit(unsigned char addr)
{
		GPIOB->ODR &= ~(1U<<0);
	
		GPIOA->ODR = (GPIOA->ODR & 0x0F) | (addr & 0xF0);
		LCD_Enable();
		Delay_ms(5);

		GPIOA->ODR = ((GPIOA->ODR & 0x0F) | (addr << 4));
		LCD_Enable();
		Delay_us(100);
}
// LCD enable
void LCD_Enable(void)
{
		GPIOB->ODR |= (1<<1);
		Delay_us(1);
		GPIOB->ODR &= ~(1U<<1);
		Delay_ms(2);
}
// gui cac lenh
void LCD_Send_Command(unsigned char addr)
{
		GPIOB->ODR &= ~(1U<<0);
	
		GPIOA->ODR = (GPIOA->ODR & 0x0F) | (addr & 0xF0);
		LCD_Enable();

		GPIOA->ODR = ((GPIOA->ODR & 0x0F) | (addr << 4));
		LCD_Enable();
	
}
// hien thi 1 ki tu
void LCD_Send_Data(unsigned char data)
{
		GPIOB->ODR |= (1U<<0);
	
		GPIOA->ODR = (GPIOA->ODR & 0x0F) | (data & 0xF0);
		LCD_Enable();
	
		GPIOA->ODR = ((GPIOA->ODR & 0x0F) | (data << 4));
		LCD_Enable();
}
// GPIO cho led LCD
void GPIO_Init_Pin(void)
{
		RCC->APB2ENR |= 1<<3 | 1 << 2;
		GPIOA->CRL |= 0x1111 << 16;
		GPIOB->CRL |= 0x11;
}


void Led_Init(void)
{
		// GPIO C - pin pc13 - 50MHz
		RCC->APB2ENR |= 1 << 4;
		GPIOC->CRH |= 3 << 20;
		// tat led
		GPIOC->ODR |= 1 << 13;
}
// de lay
void Delay_us(uint32_t us) {
    for (uint32_t i = 0; i < us * 8; i++) {
        __NOP(); // Assuming each __NOP() takes 125ns on a 64MHz clock
    }
}

void Delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        Delay_us(1000);
    }
}
void LCD_Clear(void)
{
		LCD_Send_Command(0x01);
}
// hien thi 1 string
void LCD_Send_String(char* str) {
   for (unsigned int i = 0; str[i] != 0; i++)
	{
			LCD_Send_Data(str[i]);
	}
}
