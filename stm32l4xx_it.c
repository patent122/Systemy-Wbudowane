/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */



/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */



/* USER CODE END TD */



/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define JOY_PORT GPIOE
#define JOY_PORT_PUSH GPIO_PIN_15
#define JOY_PUSH_STATE ((~(GPIOE->IDR) & JOY_PORT_PUSH) ? 1 : 0)



#define SEGMENT_A_PIN  GPIO_PIN_0
#define SEGMENT_B_PIN  GPIO_PIN_1
#define SEGMENT_C_PIN  GPIO_PIN_2
#define SEGMENT_D_PIN  GPIO_PIN_3
#define SEGMENT_E_PIN  GPIO_PIN_4
#define SEGMENT_F_PIN  GPIO_PIN_5
#define SEGMENT_G_PIN  GPIO_PIN_6
#define SEGMENT_DP_PIN GPIO_PIN_9



#define SEGMENT_GPIO_PORT GPIOG
#define SEGMENT_MASK SEGMENT_A_PIN | SEGMENT_B_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN | SEGMENT_E_PIN | SEGMENT_F_PIN | SEGMENT_G_PIN | SEGMENT_DP_PIN



#define DIGIT_1_PIN GPIO_PIN_2
#define DIGIT_2_PIN GPIO_PIN_3
#define DIGIT_3_PIN GPIO_PIN_4
#define DIGIT_4_PIN GPIO_PIN_5



#define DIGIT_GPIO_PORT GPIOB
#define DIGIT_MASK DIG1_PIN | DIG2_PIN | DIG3_PIN | DIG4_PIN
/* USER CODE END PD */



/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */



/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */



const uint8_t segments[] = {
    SEGMENT_A_PIN | SEGMENT_B_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN | SEGMENT_E_PIN | SEGMENT_F_PIN,                 // 0
    SEGMENT_B_PIN | SEGMENT_C_PIN,                                                                                 // 1
    SEGMENT_A_PIN | SEGMENT_E_PIN | SEGMENT_B_PIN | SEGMENT_D_PIN | SEGMENT_G_PIN,                                 // 2
    SEGMENT_A_PIN | SEGMENT_B_PIN | SEGMENT_G_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN,                                 // 3
    SEGMENT_F_PIN | SEGMENT_G_PIN | SEGMENT_B_PIN | SEGMENT_C_PIN,                                                 // 4
    SEGMENT_A_PIN | SEGMENT_F_PIN | SEGMENT_G_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN,                                 // 5
    SEGMENT_A_PIN | SEGMENT_F_PIN | SEGMENT_G_PIN | SEGMENT_E_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN,                 // 6
    SEGMENT_A_PIN | SEGMENT_B_PIN | SEGMENT_C_PIN,                                                                 // 7
    SEGMENT_A_PIN | SEGMENT_B_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN | SEGMENT_E_PIN | SEGMENT_F_PIN | SEGMENT_G_PIN, // 8
    SEGMENT_A_PIN | SEGMENT_B_PIN | SEGMENT_C_PIN | SEGMENT_D_PIN | SEGMENT_F_PIN | SEGMENT_G_PIN                  // 9
};



uint8_t seconds = 55;
uint8_t minutes = 59;
uint8_t hours = 23;
uint8_t mode = 0;
uint16_t dotCounter = 0;



/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */



/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */



/* External variables --------------------------------------------------------*/



/* USER CODE BEGIN EV */



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
  * @brief This function handles Prefetch fault, memory access fault.
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



static uint8_t joyPushed = 0;
  static uint8_t Digit = 0;



  static uint16_t tick = 0;




  if (tick++ == 1000) //1000ms = 1s
  {
      seconds += 1;
      tick = 0;
  }



  if (seconds >= 60)
  {
      seconds = 0;
      minutes += 1;
  }



  if (minutes >= 60)
  {
      minutes = 0;
      hours++;
  }



  if (hours >= 24)
  {
      seconds = 0;
      minutes = 0;
      hours = 0;
  }



  if (JOY_PUSH_STATE)
  {
      if (0 == joyPushed)
      {
          if (mode == 0)
              mode = 1;
          else
          {
              mode = 0;
              dotCounter = 0;
          }



          joyPushed = 1;
      }
  }
  else
      joyPushed = 0;


  if (mode == 0)
  {
      // Digit 1
      if (0 == Digit)
      {
          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_4_PIN, GPIO_PIN_RESET);



          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments[minutes / 10], GPIO_PIN_SET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_1_PIN, GPIO_PIN_SET);
      }



      // Digit 2
      if (1 == Digit)
      {
          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_1_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, GPIO_PIN_9, GPIO_PIN_RESET);



          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments[minutes % 10], GPIO_PIN_SET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_2_PIN, GPIO_PIN_SET);
          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, GPIO_PIN_9, GPIO_PIN_SET);
      }



      // Digit 3
      if (2 == Digit)
      {
          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_2_PIN, GPIO_PIN_RESET);



          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments[seconds / 10], GPIO_PIN_SET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_3_PIN, GPIO_PIN_SET);
      }



      // Digit 4
      if (3 == Digit)
      {
          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_3_PIN, GPIO_PIN_RESET);



          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments[seconds % 10], GPIO_PIN_SET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_4_PIN, GPIO_PIN_SET);
      }
  }


  if (mode == 1)
  {
      // Digit 1
      if (0 == Digit)
      {
          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_4_PIN, GPIO_PIN_RESET);



          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments[hours / 10], GPIO_PIN_SET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_1_PIN, GPIO_PIN_SET);
      }



      // Digit 2
      if (1 == Digit)
      {
          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_1_PIN, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, GPIO_PIN_9, GPIO_PIN_RESET);



          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments[hours % 10], GPIO_PIN_SET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_2_PIN, GPIO_PIN_SET);
          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, GPIO_PIN_9, GPIO_PIN_SET);
      }



      // Digit 3
      if (2 == Digit)
      {
          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_2_PIN, GPIO_PIN_RESET);



          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments[minutes / 10], GPIO_PIN_SET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_3_PIN, GPIO_PIN_SET);
      }



      // Digit 4
      if (3 == Digit)
      {
          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, SEGMENT_MASK, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_3_PIN, GPIO_PIN_RESET);



          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT,
                  dotCounter < 1000
                  ? segments[minutes % 10]
                  : segments[minutes % 10] | SEGMENT_DP_PIN
                    , GPIO_PIN_SET);




          HAL_GPIO_WritePin(SEGMENT_GPIO_PORT, segments[minutes % 10], GPIO_PIN_SET);



          HAL_GPIO_WritePin(DIGIT_GPIO_PORT, DIGIT_4_PIN, GPIO_PIN_SET);



      }
  }



  const static uint16_t freq = 100; //hz
  const static uint16_t displayTime = 100000 / freq; //ms
  static uint16_t delay = 0;



  if (delay++ == displayTime)
  {
      delay = 0;



      if (++Digit >= 4)
            Digit = 0;       //wskazanie na pierwszą cyfrę
  }



  if (dotCounter++ >= 2000)
      dotCounter = 0;


  /* USER CODE END SysTick_IRQn 1 */
}



/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/



/* USER CODE BEGIN 1 */



/* USER CODE END 1 */
