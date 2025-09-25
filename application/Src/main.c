/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	
		
		software for game device controllers
    Copyright (C) 2020  Yury Vostrenkov (yuvostrenkov@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
		
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "periphery.h"
#include "config.h"
#include "analog.h"
#include "buttons.h"
#include "leds.h"
#include "encoders.h"

#include "usb_hw.h"
#include "usb_lib.h"
#include "usb_pwr.h"

#define DEBUG

/* Private variables ---------------------------------------------------------*/
dev_config_t dev_config;
volatile uint8_t bootloader = 0;

/* Private function prototypes -----------------------------------------------*/
#ifdef DEBUG
static void LED_Init(void);
static void LED_Toggle(void);
static void LED_Off(void);
static void LED_On(void);
#endif // DEBUG

void PowerControlPin_Init(void);
static inline void Power_LatchOn(void);
static inline void Power_Release(void);

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{	
	
	// Relocate vector table
	WRITE_REG(SCB->VTOR, 0x8002000);
	
	SysTick_Init();
	
	// getting configuration from flash memory
	DevConfigGet(&dev_config);
	
	
	// set default config at first startup
	if ((dev_config.firmware_version & 0xFFF0) != (FIRMWARE_VERSION &0xFFF0))
	{
		DevConfigSet((dev_config_t *) &init_config);
		DevConfigGet(&dev_config);
	}
	AppConfigInit(&dev_config);

	
	USB_HW_Init();
	// wait for USB initialization
	Delay_ms(150);
	
	IO_Init(&dev_config);
	
	EncodersInit(&dev_config);	
	ShiftRegistersInit(&dev_config);
	RadioButtons_Init(&dev_config);
	SequentialButtons_Init(&dev_config);		
	
	// init sensors
	AxesInit(&dev_config);
	// start sequential periphery reading
	Timers_Init(&dev_config);		
	
#ifdef DEBUG
	LED_Init();
	LED_On();
#endif // DEBUG

	Delay_ms(100);

	PowerControlPin_Init();
	Power_LatchOn();
	uint16_t powerOffDelayCounter = 0;
	power_button_event_t pwr_event = PWR_BTN_EVENT_NONE;
	uint16_t timer = 0;
	
#ifdef DEBUG
	LED_Off();
#endif // DEBUG

	Delay_ms(100);

	/* PC0 pin for detecting usb c orientation. Not sure on functionality yet but it should either set or reset PC1 - DX_SEL
		
		Configure PC0 as input with pull-up
		Configure PC1 as output push-pull

		if PC0 is low (connected to GND) then set PC1 high (DX_SEL = 1)
		if PC0 is high (connected to VBUS) then set PC1 low (DX_SEL = 0)
		
		*/
	
	while (1)
	{	
#ifdef DEBUG
		if (timer <= 0)
		{
			LED_Toggle();
			timer = (PWR_OFF_DELAY_MS / 6);
		}
		timer--;
#endif // DEBUG

		(powerOffDelayCounter >= PWR_OFF_DELAY_MS) ? (powerOffDelayCounter = PWR_OFF_DELAY_MS + 1) : (powerOffDelayCounter+=3);
	
		pwr_event = ButtonsDebounceProcessPowerBtn(&dev_config);		
		switch (pwr_event) 
		{
			case PWR_BTN_EVENT_NONE:
				break;
			case PWR_BTN_EVENT_LONG_PRESS: // Turn off the power
				if (powerOffDelayCounter <= PWR_OFF_DELAY_MS)
				{
					// do nothing
				}
				else
				{
					// Shut down functions...
					Power_Release();
				}
			break;
			default:
				break;
		}

		ButtonsDebounceProcess(&dev_config);
		ButtonsReadLogical(&dev_config);
		LEDs_PhysicalProcess(&dev_config);
		
		analog_data_t tmp[8];
		AnalogGet(NULL, tmp, NULL);
		PWM_SetFromAxis(&dev_config, tmp);
	/*
		
		// Enter flasher command received
	*/
		if (bootloader > 0)
		{
			// Disable HID report generation
			NVIC_DisableIRQ(TIM2_IRQn);
			Delay_ms(50);	// time to let HID end last transmission
			// Disable USB
			PowerOff();
			USB_HW_DeInit();
			Delay_ms(500);	
			EnterBootloader();
		}
    }
}

/**
  * @brief  Jumping to memory address corresponding bootloader program
  * @param  None
  * @retval None
  */
void EnterBootloader (void)
{
	/* Enable the power and backup interface clocks by setting the
	 * PWREN and BKPEN bits in the RCC_APB1ENR register
	 */
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);

	/* Enable write access to the backup registers and the
		* RTC.
		*/
	SET_BIT(PWR->CR, PWR_CR_DBP);
	WRITE_REG(BKP->DR4, 0x424C);
	CLEAR_BIT(PWR->CR, PWR_CR_DBP);
	
	CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);
	
	NVIC_SystemReset();
}

#ifdef DEBUG
static void LED_Init(void) 
{
	// Power output control testing
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN); // Enable GPIOC clock
	/* PC13 as output */
	GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);    /* Clear MODE13 and CNF13 fields */
	GPIOC->CRH |= GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0;  /* Set MODE13 to 3 (Output) */

}
static void LED_Toggle(void) 
{
    GPIOC->ODR ^= GPIO_ODR_ODR13;
}

static void LED_Off(void) 
{
    GPIOC->ODR &= ~(GPIO_ODR_ODR13);
}

static void LED_On(void) 
{
    GPIOC->ODR |= (GPIO_ODR_ODR13);
}
#endif // DEBUG

inline void USB_OrientationDetect(void)
{
	/* PC0 pin for detecting usb c orientation. Not sure on functionality yet but it should either set or reset PC1 - DX_SEL

	Configure PC0 as input with pull-down since connected to Rp on CC line
	Configure PC1 as output push-pull

	if PC0 is low (connected to GND) then set PC1 high (DX_SEL = 1)
	if PC0 is high (connected to VBUS) then set PC1 low (DX_SEL = 0)
	*/

	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN); // Enable GPIOC clock

	GPIOC->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);    /* Clear MODE0 and CNF0 fields */

}

void PowerControlPin_Init(void)
{
	// TODO: These pins are for the breakout board only, before flashing to PCB ensure all pins are correct
	
	// Step 1: Enable GPIOA clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable GPIOA clock

	// Step 2: Configure PA10 as push-pull output
	// PA10 is controlled by CRH (Configuration Register High, for pins 8-15)
	GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10); // Clear CNF and MODE bits for PA10
	GPIOA->CRH |= GPIO_CRH_MODE10_0; // Set MODE10 to 01 (output, 10 MHz speed)
	// CNF10 is 00 by default after clearing (push-pull output)

	// Step 3: Optional - Set initial state (e.g., high as it will be linked to N-MOS)
	GPIOA->BSRR = GPIO_BSRR_BS10; // Reset PA10 (set to high)

}

static inline void Power_LatchOn(void) { GPIOA->BSRR = GPIO_BSRR_BR10; }
static inline void Power_Release(void) { GPIOA->BSRR = GPIO_BSRR_BS10; }

/**
  * @}
  */

/**
  * @}
  */