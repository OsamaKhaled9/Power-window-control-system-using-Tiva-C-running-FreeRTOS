#ifndef dio_h
#define dio_h


/*************************************************************
*                           Includes
**************************************************************/
#include "Types.h"
#include "tm4c123gh6pm.h"

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/
#define NUM_OF_PORTS           6
#define NUM_OF_PINS_PER_PORT   8

#define PORTA_ID               0
#define PORTB_ID               1
#define PORTC_ID               2
#define PORTD_ID               3
#define PORTE_ID               4
#define PORTF_ID               5

#define GPIO_PIN_O      0x00000001      // GPIO pin  O
#define GPTO_PIN_1      0x00000002      // GPIO pin  1
#define GPIO_PIN_2      0x00000004      // GPIO pin  2
#define GPIO_PIN_3      0x00000008      // GPIO pin  3
#define GPIO_PIN_4      0x00000010      // GPIO pin  4
#define GPIO_PIN_5      0x00000020      // GPIO pin  5
#define GPIO_PIN_6      0x00000040      // GPIO pin  6
#define GPIO_PIN_7      0x00000080      // GPIO pin  7


void delay_ms(int n);
/************************************************************
*Function:DIO_inititialization
*Parameters:Void
*Description : This function initialize the I/O of the microcontroller 
*Return_Value : None 
*************************************************************/
void DIO_Init(void);


/************************************************************
*Function:DIO_Write Pin
*Parameters:Pointer of the port , pin number , value to write
*Description : This function initialize the I/O of the microcontroller 
* Return Value : None 
*************************************************************/
void DIO_WritePin(uint32_vol_ptr Port, uint8 pin, uint8 value);





/************************************************************
*Function:DIO_Write Port
*Parameters:Void
*Description : This function initialize the I/O of the microcontroller 
* Return Value : None 

*************************************************************/
void DIO_WritePort(uint32_vol_ptr Port, uint8 value);



/************************************************************
*Function:DIO_Read Pin
*Parameters:Port ID and Pin number to read
*Description :  Read and return the value for the required pin, it should be Logic High or Logic Low.
               If the input port number or pin number are not correct, The function will return Logic Low.
* Return Value : Pin Value 
*************************************************************/

uint8 DIO_ReadPin(uint8 Port_ID, uint8 pin_num);



/************************************************************
*Function:DIO_Read Port
*Parameters:Port Num
*Description : Read and return the value of the required port.
              If the input port number is not correct, The function will return ZERO value.
* Return Value : Port value  
*************************************************************/

uint32 DIO_ReadPort(uint8 Port_ID);

	void PORTA_INIT(void);
 void PORTF_INIT(void);
	void PORTB_INIT(void);
	void PORTD_INIT(void);



#endif
