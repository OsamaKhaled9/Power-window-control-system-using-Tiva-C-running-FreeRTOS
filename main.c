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
#define Jam_mode		 	   0
#define Motor_Forward 	 1
#define Motor_Backward 	 2	
#define Motor_OFF				 3
static uint16_t mode = 	 3;  	      // Global Variable To be sent using Queue 

#define GPIO_PIN_0   0
#define MOTOR_PIN_1 (1U << 2)   	  // Motor control pin 1 (PB0)
#define MOTOR_PIN_2 (1U << 3)    	 // Motor control pin 2 (PB1)
#define SW1							 0	



void ManualControlTask(void* parameters);
void MotorControlTask( void *pvParameters );
void JammerTask(void *pvParameters);

void motorForward(void);        // Motor_Forward Function Prototype 
void motorReverse(void);			 // Motor_Reverse  Function Prototype 
void motorOFF(void);
void delayMs(int n);

/*Semaphores*/
SemaphoreHandle_t xJamAutoSemaphore;
SemaphoreHandle_t xJamPressedSemaphore;
xSemaphoreHandle xMutex;


// Define task handles
xTaskHandle switchTaskHandle;
xTaskHandle functionTaskHandle;
xTaskHandle passenger;

// Define Queue handles
xQueueHandle xMotorQueue;

//Global Variables
static int x=0;

/*------------------Functions----------*/
void delayMs(int n){
  int i,j;
  for(i=0;i<n;i++)
  {
    for(j=0;j<3180;j++)
    {}
  }
}

void motorForward() {
    // Set motor control pins for forward motion 1 0
    GPIO_PORTA_DATA_R = (GPIO_PORTA_DATA_R & ~MOTOR_PIN_2) | MOTOR_PIN_1;
}

void motorReverse() {
    // Set motor control pins for reverse motion 0 1
    GPIO_PORTA_DATA_R = (GPIO_PORTA_DATA_R & ~MOTOR_PIN_1) | MOTOR_PIN_2;
}

void motorOFF()
{
			//Set The control Pins both to 0
	    GPIO_PORTA_DATA_R &= ~(MOTOR_PIN_1 | MOTOR_PIN_2);

}


// Polling task function checking for all inputs
void ManualControlTask(void* parameters) {
	portBASE_TYPE xStatus;
    while (1) {
if ((GPIO_PORTB_DATA_R & 0x01) == 0x01) {   //auto + manual up PB0  driverrr
		vTaskDelay(pdMS_TO_TICKS(4000));
   // delayMs(500);
			 GPIOF->DATA ^= 0x02;

    if ((GPIO_PORTB_DATA_R & 0x01) == 0x01) {
        while ((GPIO_PORTB_DATA_R & 0x01) == 0x01) { 
            xStatus = xQueueSendToBack(xMotorQueue, &(uint32_t){Motor_Forward}, 0);
            if ((GPIO_PORTB_DATA_R & 0x01) != 0x01) {
                xStatus = xQueueSendToBack(xMotorQueue, &(uint32_t){Motor_OFF}, 0);
                break;
            }
        }
        
    }
		else {
            xSemaphoreGive(xJamAutoSemaphore);
            xStatus = xQueueSendToBack(xMotorQueue, &(uint32_t){Motor_Forward}, 0);
        }
}

if ((GPIO_PORTB_DATA_R & 0x02) == 0x02) {   //auto + manual down PB1  driverrr
	vTaskDelay(pdMS_TO_TICKS(4000));
    //delayMs(100000000);
    if ((GPIO_PORTB_DATA_R & 0x02) == 0x02) {
        while ((GPIO_PORTB_DATA_R & 0x02) == 0x02) { 
            xStatus = xQueueSendToBack(xMotorQueue, &(uint32_t){Motor_Backward}, 0);
            if ((GPIO_PORTB_DATA_R & 0x02) != 0x02) {
                xStatus = xQueueSendToBack(xMotorQueue, &(uint32_t){Motor_OFF}, 0);
                break;
            }
        }
        
    }
		else {
           
            xStatus = xQueueSendToBack(xMotorQueue, &(uint32_t){Motor_Backward}, 0);
        }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
				
				if(((GPIO_PORTB_DATA_R & 0x10)==0x10) && ((GPIO_PORTA_DATA_R & 0x40) != 0x40)){  //pb4(jam) and PA6 off/on
									vTaskDelay(pdMS_TO_TICKS(4000));

//						delayMs(500);
				if(((GPIO_PORTB_DATA_R & 0x10)==0x10) && ((GPIO_PORTA_DATA_R & 0x40) != 0x40)){
					while (((GPIO_PORTB_DATA_R & 0x10) == 0x10) && ((GPIO_PORTA_DATA_R & 0x40) != 0x40)){
							xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Motor_Forward},0);
						if((GPIO_PORTB_DATA_R & 0x10)!=0x10){ 
							xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Motor_OFF},0);
							break;
					}
									}
		}
				else{
					xSemaphoreGive(xJamAutoSemaphore);
					xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Motor_Forward},0);
				}
	}
//////////////////////////////////////////////////////////////////////////
			if(((GPIO_PORTB_DATA_R & 0x20)==0x20) && ((GPIO_PORTA_DATA_R & 0x40) != 0x40)){  //pb4(jam) and PA6 off/on
				vTaskDelay(pdMS_TO_TICKS(4000));
				GPIOF->DATA ^= 0x04;

//						delayMs(500);
				if(((GPIO_PORTB_DATA_R & 0x20)==0x20) && ((GPIO_PORTA_DATA_R & 0x40) != 0x40)){
					while (((GPIO_PORTB_DATA_R & 0x20) == 0x20) && ((GPIO_PORTA_DATA_R & 0x40) != 0x40)){
							xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Motor_Forward},0);
						if((GPIO_PORTB_DATA_R & 0x20)!=0x20){ 
							xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Motor_OFF},0);
							break;
					}
									}
		}
				else{
					xSemaphoreGive(xJamAutoSemaphore);
					xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){Motor_Forward},0);
				}
	}
			//////////////////////////////////////////////////////////////////////////////
	/*if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            // Check if PB4 and PB0 are pressed together
            if (((GPIO_PORTB_DATA_R & 0x10) == 0x10) && ((GPIO_PORTB_DATA_R & 0x01) == 0x01)) {
                // Release mutex
                xSemaphoreGive(xMutex);
                // Execute the code for PB4 and PB0 pressed together
                // ...
                vTaskDelay(pdMS_TO_TICKS(4000));
                if (((GPIO_PORTB_DATA_R & 0x10) == 0x10) && ((GPIO_PORTB_DATA_R & 0x01) == 0x01)) {
                    while (((GPIO_PORTB_DATA_R & 0x10) == 0x10) && ((GPIO_PORTB_DATA_R & 0x01) == 0x01)) {
                        xStatus = xQueueSendToBack(xMotorQueue, &(uint32_t){Motor_Forward}, 0);
                        if (((GPIO_PORTB_DATA_R & 0x10) != 0x10) || ((GPIO_PORTB_DATA_R & 0x01) != 0x01)) {
                            xStatus = xQueueSendToBack(xMotorQueue, &(uint32_t){Motor_OFF}, 0);
                            break;
                        }
                    }
                } else {
                    xSemaphoreGive(xJamAutoSemaphore);
                    xStatus = xQueueSendToBack(xMotorQueue, &(uint32_t){Motor_Forward}, 0);
                }
            } else {
                // Release mutex
                xSemaphoreGive(xMutex);
                // Execute the rest of the code
                // ...
                // Your existing code for handling other cases
            }
        } else {
            // Failed to take mutex, handle error
            // ...
        }*/
        vTaskDelay(pdMS_TO_TICKS(10)); // debounce switch
    }
}


/*-------------------Jammer--------*/
void JammerTask(void *pvParameters){
	long message=(long)pvParameters;
	portBASE_TYPE xStatus;
	xSemaphoreTake(xJamAutoSemaphore,0);
	for(;;)
	{
			xSemaphoreTake(xJamAutoSemaphore,portMAX_DELAY);
		  xSemaphoreTake(xJamPressedSemaphore,portMAX_DELAY);
			xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){0},0);
	}
}
/*-----------MotorController-------*/
void MotorControlTask(void *pvParameters){
	uint32_t message;
	for(;;)
{
	xQueueReceive(xMotorQueue,&message,portMAX_DELAY);
	
	switch(message){
		case Jam_mode :
			motorOFF();
				vTaskDelay(pdMS_TO_TICKS(4000));

			//delayMs(1000);
			motorReverse();   	//Jamm Mode Motor stops then move backwards
			vTaskDelay(pdMS_TO_TICKS(4000));

			//delayMs(1000);
			motorOFF();
			break;
		case Motor_Forward:
			motorForward();    //Motor forward
			break;
		case Motor_Backward :
			motorReverse();   //Motor backward
			break;		
		case Motor_OFF:
			motorOFF(); 				//Motor Stops
			break;
		}
	}
}

                         
int main( void )
{
	PORTA_INIT();
  PORTF_INIT();
	PORTB_INIT();
	PORTD_INIT();
	
	xMotorQueue=xQueueCreate(20,sizeof(uint32_t));	
	xJamAutoSemaphore = xSemaphoreCreateBinary();
	xJamPressedSemaphore = xSemaphoreCreateBinary();
	xMutex = xSemaphoreCreateMutex();
	__ASM("CPSIE i");
	if( (xJamAutoSemaphore && xJamPressedSemaphore && xMotorQueue && xMutex) != NULL )
		{
			
			xTaskCreate(ManualControlTask, "Switch Task", configMINIMAL_STACK_SIZE, NULL, 1, switchTaskHandle);
			xTaskCreate( MotorControlTask, "Motor",configMINIMAL_STACK_SIZE, NULL, 2, NULL );
			xTaskCreate (JammerTask, "Jammer", configMINIMAL_STACK_SIZE,NULL,3,NULL);
			
			
			/* Start the scheduler so the created tasks start executing. */
			vTaskStartScheduler();
		}

    for( ;; );
}

void GPIOF_Handler(void){  	//Port-F handler

//--------------------------Jammer Button--------------------
	 if(GPIO_PORTF_MIS_R & (1 << 0)){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xJamPressedSemaphore,&xHigherPriorityTaskWoken);
	GPIOF->DATA ^= 0x08;
	GPIOF->ICR = 0xff;       				 										// clear the interrupt flag of PORTF 
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  }
}

/*----------------------------Port-D handler--------------------------------------------*/
void GPIOD_Handler(void){
/*--------------------------Limit Switch up Button--------------------*/
	 if(GPIO_PORTD_MIS_R & (1 << 0)){
		 GPIOF->DATA ^= 0x08;
		 GPIOD->ICR = 0xff;  
      xQueueSendFromISR(xMotorQueue,&(uint32_t){Motor_OFF},0);	
  }
/*----------------------------Limit Switch down Button-------------------*/
	 if (GPIO_PORTD_MIS_R & (1 << 1)){
		 GPIOF->DATA ^= 0x02;
		 GPIOD->ICR = 0xff;  
			xQueueSendFromISR(xMotorQueue,&(uint32_t){Motor_OFF},0);	
 }
}