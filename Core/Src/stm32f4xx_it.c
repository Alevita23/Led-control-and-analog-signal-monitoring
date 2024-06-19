/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint32_t buttonPressCount = 0;
uint32_t buttonPressStartTime = 0;
extern time;
uint16_t time1=0;
extern state;
extern cont;
int c=0;
int flag=0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1)
	{
	}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if(state==0){//led spento
		time=0;
		//char buff[20]; // Buffer per contenere la stringa convertita
		//sprintf(buff, "Time: %lu\r\n", time); // Formatta il numero in una stringa
		//HAL_UART_Transmit(&huart2, (uint8_t *)buff, strlen(buff), 1000);
		if (flag==0){
			char msg[] = "Timer start\r\n";
			HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
			HAL_TIM_Base_Start_IT(&htim2); // avvio il timer appena premo il pulsante
			__HAL_TIM_SET_COUNTER(&htim2, 0); //lo faccio partire da 0
			//char buff[20]; // Buffer per contenere la stringa convertita
			//sprintf(buff, "Time: %lu\r\n", time); // Formatta il numero in una stringa
			//HAL_UART_Transmit(&huart2, (uint8_t *)buff, strlen(buff), 1000);
			flag=1;
		}
		else if(flag==1){
			char msgt[] = "Timer stopped\r\n";
			HAL_UART_Transmit(&huart2, (uint8_t *)msgt, strlen(msgt), 1000);
			HAL_TIM_Base_Stop_IT(&htim2); // blocco il timer
			time=__HAL_TIM_GET_COUNTER(&htim2); //registra il valore del time

			//char buff[20]; // Buffer per contenere la stringa convertita
			//sprintf(buff, "Time: %lu\r\n", time); // Formatta il numero in una stringa
			//HAL_UART_Transmit(&huart2, (uint8_t *)buff, strlen(buff), 1000);
			time1=(time*999)/799999;
			//char bufff[20]; // Buffer per contenere la stringa convertita
			//sprintf(bufff, "Tempo: %lu\r\n", time1); // Formatta il numero in una stringa
			//HAL_UART_Transmit(&huart2, (uint16_t *)bufff, strlen(bufff), 1000);

			if (time1>=100){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				char msgt[] = "LED on\r\n";
				HAL_UART_Transmit(&huart2, (uint8_t *)msgt, strlen(msgt), 1000);
				c=1;
				flag=0;
			}
			else {
				flag=0;
			}

		}
	}

	else if(state==1){ //se il led è acceso
		cont++; // conta il numero di volte che il pulsante è stato premuto
		if(c==1){
			HAL_TIM_Base_Start_IT(&htim2); // avvio il timer appena premo il pulsante
			__HAL_TIM_SET_COUNTER(&htim2, 0); //lo faccio partire da 0
			c=0; //cosi il timer non si avvia di nuovo
		}
		else {
		}

		if (cont==10){// se il tasto è stato premuto 5 volte, 10 perchè ogni volta che si preme il pulsante si generano due interrupt

			HAL_TIM_Base_Stop_IT(&htim2); // blocco il timer
			time=__HAL_TIM_GET_COUNTER(&htim2);
			if(((time*999)/799999)<=400){ // se sono passati meno di 4 secondi
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);//spegni il led
				char msgt[] = "LED off\r\n";
				HAL_UART_Transmit(&huart2, (uint8_t *)msgt, strlen(msgt), 1000);
				cont=0;// azzera il contatore
				//char message1[] = "LED spento\r\n";
				//HAL_UART_Transmit(&huart2, (uint8_t *)message, sizeof(message) - 1, HAL_MAX_DELAY);
			}
			else{
				c=1;// se sono passati più di 4 secondi permetti di riavviare il timer
				cont=0;// azzera il contatore
			}
		}
	}

/* USER CODE END EXTI15_10_IRQn 0 */
HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
/* USER CODE BEGIN EXTI15_10_IRQn 1 */

/* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
