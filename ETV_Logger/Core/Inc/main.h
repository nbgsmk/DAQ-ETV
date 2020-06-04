/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	hardwareOut,
	email,
	udp,
	snmp
} notifikacija_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_ULAZA_max 6
#define ADC_ULAZA_koristi 6
#define ADC_ULAZA_internih 2
#define ADC_SAMPLES_max 8
#define ADC_SAMPLES_koristi 8
#define ADC_REPEAT_PERIOD_mS 10000
#define a_dummy -99
#define b_dummy -99
#define c_dummy -99
#define DIGITAL_ULAZA_max 3
#define DIGITAL_ULAZA_koristi 3
#define DIGITAL_SAMPLES_max 5
#define DIGITAL_SAMPLES_koristi 3
#define DIGITAL_REPEAT_PERIOD_mS 120000
#define d_dummy -99
#define e_dummy -99
#define blinkiPERIOD 1000
#define f_dummy -998
#define EXTI_MINIMUM_REPEAT_mS 500
#define BOARD_LED_Pin GPIO_PIN_13
#define BOARD_LED_GPIO_Port GPIOC
#define ANALOG_in_A_Pin GPIO_PIN_4
#define ANALOG_in_A_GPIO_Port GPIOA
#define ANALOG_in_B_Pin GPIO_PIN_5
#define ANALOG_in_B_GPIO_Port GPIOA
#define ANALOG_in_C_Pin GPIO_PIN_6
#define ANALOG_in_C_GPIO_Port GPIOA
#define ANALOG_in_D_Pin GPIO_PIN_7
#define ANALOG_in_D_GPIO_Port GPIOA
#define EXT_interrupt_A_Pin GPIO_PIN_0
#define EXT_interrupt_A_GPIO_Port GPIOB
#define EXT_interrupt_A_EXTI_IRQn EXTI0_IRQn
#define EXT_interrupt_B_Pin GPIO_PIN_1
#define EXT_interrupt_B_GPIO_Port GPIOB
#define EXT_interrupt_B_EXTI_IRQn EXTI1_IRQn
#define DIGITAL_out_A_Pin GPIO_PIN_2
#define DIGITAL_out_A_GPIO_Port GPIOB
#define DIGITAL_out_B_Pin GPIO_PIN_10
#define DIGITAL_out_B_GPIO_Port GPIOB
#define _USB_DRIVE_VBUS_FS_z_freeze_Pin GPIO_PIN_10
#define _USB_DRIVE_VBUS_FS_z_freeze_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
