#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "TM4C123GH6PM.h"
#include "DIO.h"
#include "types.h"
#include "bitwise_operations.h"
void delay_ms(int n){
  for(int i = 0 ; i<n;i++)
  {
    for(int j = 0 ; j<3160; j++)
    {}
  }
}


void DIO_Init()
{
SYSCTL_RCGCGPIO_R |= 0x20;
while(( SYSCTL_PRGPIO_R & 0x20 ) == 0 ) {}
GPIO_PORTF_LOCK_R = 0X4C4F434B;
GPIO_PORTF_CR_R = 0x1F;
//GPIO_PORTF_DIR_R &= ~(0x11);
GPIO_PORTF_DIR_R |= 0x0E;
GPIO_PORTF_PUR_R = 0x11;
GPIO_PORTF_DEN_R |= 0x1F; 
}  


void DIO_WritePin(uint32_vol_ptr Port, uint8 pin, uint8 value)
{
  if(value == 0)
  {
      Clear_Bit(*Port,pin);
  }
  else if(value == 1)
  {     
    SetBit(*Port,pin);
  }
}


void DIO_WritePort(uint32_vol_ptr Port, uint8 value)
{
  *Port = value ; 
} 

uint8 DIO_ReadPin(uint8 Port_ID, uint8 pin_num)
{
	uint8 pin_value = LOGIC_LOW;

	/*
	 * Check if the input port number is greater than NUM_OF_PINS_PER_PORT value.
	 * Or if the input pin number is greater than NUM_OF_PINS_PER_PORT value.
	 * In this case the input is not valid port/pin number
	 */
	if((pin_num >= NUM_OF_PINS_PER_PORT) || (Port_ID >= NUM_OF_PORTS))
	{
		/* Do Nothing */
	}
	else
	{
		/* Read the pin value as required */
		switch(Port_ID)
		{
		case PORTA_ID:
			if(GET_BIT(GPIO_PORTA_DATA_R,pin_num))
			{
				pin_value = LOGIC_HIGH;
			}
			else
			{
				pin_value = LOGIC_LOW;
			}
			break;
		case PORTB_ID:
			if(GET_BIT(GPIO_PORTB_DATA_R,pin_num))
			{
				pin_value = LOGIC_HIGH;
			}
			else
			{
				pin_value = LOGIC_LOW;
			}
			break;
		case PORTC_ID:
			if(GET_BIT(GPIO_PORTC_DATA_R,pin_num))
			{
				pin_value = LOGIC_HIGH;
			}
			else
			{
				pin_value = LOGIC_LOW;
			}
			break;
		case PORTD_ID:
			if(GET_BIT(GPIO_PORTD_DATA_R,pin_num))
			{
				pin_value = LOGIC_HIGH;
			}
			else
			{
				pin_value = LOGIC_LOW;
			}
			break;
                        
                        
                case PORTF_ID:
			if(GET_BIT(GPIO_PORTF_DATA_R,pin_num))
			{
				pin_value = LOGIC_HIGH;
			}
			else
			{
				pin_value = LOGIC_LOW;
			}
			break;
		}
	}

	return pin_value;
}


uint32 DIO_ReadPort(uint8 Port_ID)
{
  uint32 value = LOGIC_LOW;

	/*
	 * Check if the input number is greater than NUM_OF_PORTS value.
	 * In this case the input is not valid port number
	 */
	if(Port_ID >= NUM_OF_PORTS)
	{
		/* Do Nothing */
	}
	else
	{
		/* Read the port value as required */
		switch(Port_ID)
		{
		case PORTA_ID:
			value = GPIO_PORTA_DATA_R;
			break;
		case PORTB_ID:
			value = GPIO_PORTB_DATA_R;
			break;
		case PORTC_ID:
			value = GPIO_PORTC_DATA_R;
			break;
		case PORTD_ID:
			value = GPIO_PORTD_DATA_R;
			break;
                 case PORTF_ID:
			value = GPIO_PORTF_DATA_R;
			break;       
                        
		}
	}

	return value;
}

  void PORTF_INIT(void){ 
  SYSCTL->RCGCGPIO |= 0x00000020;    // 1) F clock
  GPIOF->LOCK = 0x4C4F434B;  				 // 2) unlock PortF PF0  
  GPIOF->CR = 0x1F;          				 // allow changes to PF4-0       
  GPIOF->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOF->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOF->DIR = 0x0E;         				 // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIOF->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOF->PUR = 0x11;       				   // enable pullup resistors on PF4,PF0       
  GPIOF->DEN = 0x1F;       				   // 7) enable digital pins PF4-PF0
	GPIOF->DATA = 0x00;
	GPIOF->ICR = 0x11;     // Clear any Previous Interrupt 
	GPIOF->IM |=0x11;      // Unmask the interrupts for PF0 and PF4
	GPIOF->IS |= 0x11;     // Make bits PF0 and PF4 level sensitive
	GPIOF->IEV &= ~0x11;   // Sense on Low Level
	NVIC_EN0_R |= 0x40000000;
  NVIC_PRI7_R |= 0xe0000;  // Enable the Interrupt for PortF in NVIC
}

void PORTB_INIT(void){
	SYSCTL->RCGCGPIO |= 0x00000002; 	// initialize clock                           // looping until clock is initiallized
	GPIOB->LOCK = 0x4C4F434B;
	GPIOB->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOB->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL 
	GPIOB->CR |= 0x000000ff;                                    // unlocking commit register for switch 1 & switch 2 
  GPIOB->DIR |= 0x00000000;                            // detrmining the output pins                                   
  GPIOB->AFSEL = 0x00; 
	GPIOB->DEN |= 0x000000ff;              // detrmining the pins direction 
  GPIOB->DATA |= 0x00000000;
	GPIOB -> PDR = 0xff; 	
}
void PORTD_INIT(void){

	SYSCTL->RCGCGPIO |= 0x00000008; 	// initialize clock                           // looping until clock is initiallized
	GPIOD->LOCK = 0x4C4F434B;
	GPIOD->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOD->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL 
	GPIOD->CR |= 0x000000ff;                                    // unlocking commit register for switch 1 & switch 2 
  GPIOD->DIR |= 0x00000000;                            // detrmining the output pins                                   
  GPIOD->AFSEL = 0x00; 
	GPIOD->DEN |= 0x000000ff;              // detrmining the pins direction 
  GPIOD->DATA |= 0x00000000;
	GPIOD -> PDR = 0xff; 	
	GPIOD->ICR = 0xff;     // Clear any Previous Interrupt
  GPIOD->IM |=0xff;      // Unmask the interrupts for PF0 and PF4	
	NVIC_EN0_R |= 0x08; 
	NVIC_PRI0_R |= 0xe000; 
}

void PORTA_INIT(void){
	SYSCTL_RCGCGPIO_R |= 0x00000001;                                     // initialize clock
	while((SYSCTL_PRGPIO_R&0x00000001) == 0){}                           // looping until clock is initiallized
	GPIO_PORTA_CR_R |= 0x000000ff;                                    // unlocking commit register for switch 1 & switch 2 
  GPIO_PORTA_DIR_R |= 0x0000000c;                            // detrmining the output pins                                   
  GPIO_PORTA_DEN_R |= 0x000000ff;              // detrmining the pins direction 
  GPIO_PORTA_DATA_R |= 0x00000000;
	GPIOA -> PUR = 0x80; 
}

