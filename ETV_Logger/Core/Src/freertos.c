/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "adc.h"
#include "taskovi.h"
#include "stdlib.h"
#include "rtc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint32_t adcDMABuf[ADC_ULAZA_max] = { 0 };
float adcAnalog[ADC_ULAZA_max] = { 0 };
uint32_t adcLogic[ADC_ULAZA_max] = { 0 };

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for blinki */
osThreadId_t blinkiHandle;
const osThreadAttr_t blinki_attributes = {
  .name = "blinki",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for analogMerenje */
osThreadId_t analogMerenjeHandle;
const osThreadAttr_t analogMerenje_attributes = {
  .name = "analogMerenje",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for analogFilter */
osThreadId_t analogFilterHandle;
const osThreadAttr_t analogFilter_attributes = {
  .name = "analogFilter",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for extInt_A */
osThreadId_t extInt_AHandle;
const osThreadAttr_t extInt_A_attributes = {
  .name = "extInt_A",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for extInt_B */
osThreadId_t extInt_BHandle;
const osThreadAttr_t extInt_B_attributes = {
  .name = "extInt_B",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for adcBufQue_ */
osMessageQueueId_t adcBufQue_Handle;
const osMessageQueueAttr_t adcBufQue__attributes = {
  .name = "adcBufQue_"
};
/* Definitions for adcBufSmf_ */
osSemaphoreId_t adcBufSmf_Handle;
const osSemaphoreAttr_t adcBufSmf__attributes = {
  .name = "adcBufSmf_"
};
/* Definitions for adcFinalSmf_ */
osSemaphoreId_t adcFinalSmf_Handle;
const osSemaphoreAttr_t adcFinalSmf__attributes = {
  .name = "adcFinalSmf_"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void adcBackgroundStart();
void digitalCitac(uint32_t);
void djokicaStack(uint8_t);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void blinkiStart(void *argument);
void analogMerenjeStart(void *argument);
void analogFilterStart(void *argument);
void extInt_A_receiver(void *argument);
void extInt_B_receiver(void *argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationTickHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void) {

}

__weak unsigned long getRunTimeCounterValue(void) {
	return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook(void) {
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	 to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
	 task. It is essential that code added to this hook function never attempts
	 to block in any way (for example, call xQueueReceive() with a block time
	 specified, or call vTaskDelay()). If the application makes use of the
	 vTaskDelete() API function (as this demo application does) then it is also
	 important that vApplicationIdleHook() is permitted to return to its calling
	 function, because it is the responsibility of the idle task to clean up
	 memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
__weak void vApplicationTickHook(void) {
	/* This function will be called by each tick interrupt if
	 configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
	 added here, but the tick hook is called from an interrupt context, so
	 code must not attempt to block, and only the interrupt safe FreeRTOS API
	 functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask,
		signed char *pcTaskName) {
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	 called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void) {
	/* vApplicationMallocFailedHook() will only be called if
	 configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
	 function that will get called if a call to pvPortMalloc() fails.
	 pvPortMalloc() is called internally by the kernel whenever a task, queue,
	 timer or semaphore is created. It is also called by various parts of the
	 demo application. If heap_1.c or heap_2.c are used, then the size of the
	 heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	 FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	 to query the size of free heap space that remains (although it does not
	 provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of adcBufSmf_ */
  adcBufSmf_Handle = osSemaphoreNew(1, 1, &adcBufSmf__attributes);

  /* creation of adcFinalSmf_ */
  adcFinalSmf_Handle = osSemaphoreNew(1, 1, &adcFinalSmf__attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of adcBufQue_ */
  adcBufQue_Handle = osMessageQueueNew (200, sizeof(uint32_t), &adcBufQue__attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of blinki */
  blinkiHandle = osThreadNew(blinkiStart, NULL, &blinki_attributes);

  /* creation of analogMerenje */
  analogMerenjeHandle = osThreadNew(analogMerenjeStart, NULL, &analogMerenje_attributes);

  /* creation of analogFilter */
  analogFilterHandle = osThreadNew(analogFilterStart, NULL, &analogFilter_attributes);

  /* creation of extInt_A */
  extInt_AHandle = osThreadNew(extInt_A_receiver, NULL, &extInt_A_attributes);

  /* creation of extInt_B */
  extInt_BHandle = osThreadNew(extInt_B_receiver, NULL, &extInt_B_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;) {
		osDelay(100);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_blinkiStart */
/**
 * @brief Function implementing the blinki thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_blinkiStart */
void blinkiStart(void *argument)
{
  /* USER CODE BEGIN blinkiStart */
	int blinkLen = 4;
	/* Infinite loop */
	for (;;) {
		uint32_t x = HAL_RTCEx_BKUPRead(&hrtc, 11);
		trep(blinkLen);
		osDelay(blinkiPERIOD - blinkLen+1);
	}
  /* USER CODE END blinkiStart */
}

/* USER CODE BEGIN Header_analogMerenjeStart */
/**
 * @brief Function implementing the analogMerenje thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_analogMerenjeStart */
void analogMerenjeStart(void *argument)
{
  /* USER CODE BEGIN analogMerenjeStart */
	/* Infinite loop */
	for (;;) {
		adcBackgroundStart();
		osDelay(pdMS_TO_TICKS(ADC_REPEAT_PERIOD_mS));
	}
  /* USER CODE END analogMerenjeStart */
}

/* USER CODE BEGIN Header_analogFilterStart */
/**
 * @brief Function implementing the analogFilter thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_analogFilterStart */
void analogFilterStart(void *argument)
{
  /* USER CODE BEGIN analogFilterStart */

	/*
	 * ADC ulazi su:
	 * adc1_in4
	 * adc1_in5
	 * ...
	 * temperature sensor (interni senzor na cpu)
	 * Vrefint (interni referentni napon za adc)
	 */
	uint32_t istorija[ADC_SAMPLES_max][ADC_ULAZA_max] = { 0 };
	uint32_t najnoviji[ADC_ULAZA_koristi] = { 0 };
	float analogFiltered[ADC_ULAZA_koristi] = { 0 };
	uint32_t tmp = 0;


	/* Infinite loop */
	for (;;) {
		// najnoviji rezultat[] prebacujem jedan po jedan ulaz, u matricu istorija[][]

		for (int ulaz = 0; ulaz < ADC_ULAZA_koristi; ++ulaz) {
			// sledeci rezultat merenja iz queue stavim u najnoviji[]. Moze to i brze ali ovako je cistije i lakse

			while(osOK != osMessageQueueGet(adcBufQue_Handle, &najnoviji[ulaz], NULL, osWaitForever) ) {
				/*
				 * ako istekne osWaitforever (za 1mS tick to je oko 49 dana) ili se desi bilo sta drugo
				 * vracam se i nastavljam da cekam
				 */
				// FEA ako se desilo bilo sta drugo osim sto je istekao osWaitforever?
			}

		}

		// nakon dovoljno cekanja u queue svi ulazi su izmereni i prebaceni u najnoviji[]
		// sad cemo da ih procesiramo
		for (int sempl = 0; sempl < ADC_SAMPLES_koristi; ++sempl) {
			// prebaci najnovije merenje u istoriju
			for (int ulaz = 0; ulaz < ADC_ULAZA_koristi; ++ulaz) {
				// jedan po jedan ulaz
				istorija[sempl][ulaz] = najnoviji[ulaz];

				// kad sam vec tu, za isti taj ulaz nadjem average svih semplova do sada
				tmp = 0;
				for (int i = 0; i < ADC_SAMPLES_koristi; ++i) {
					tmp = tmp + istorija[i][ulaz];
				}
				// i sacuvam to
				analogFiltered[ulaz] = ((float) tmp) / ((float) ADC_SAMPLES_koristi);
			}

		}
		// specijalan tretman za interno merenje temperature procesora
		// pogledati datasheet i reference manual za konkretan cpu, izabranu rezoluciju adc-a itd
		// celzijusa = ((adcMerenje * (Vrefint / adcRezolucija) - 0.76) / 2.5) + 25
		// celzijusa = ((adcMerenje * (  3.3   /     4096     ) - 0.76) / 2.5) + 25;
		float tmp = analogFiltered[ADC_ULAZA_koristi - 2];// prioritet internog temp senzora = pretposlednji (ali indeksi pocinju od 0)
		float cpuTemperatura = ((tmp * (3.3 / 4096) - 0.76) / 2.5) + 25;
		float cpuAdcVoltageReference = analogFiltered[ADC_ULAZA_koristi - 1];// prioritet Vrefint = poslednji (ali indeksi pocinju od 0)
		analogFiltered[ADC_ULAZA_koristi - 2] = cpuTemperatura;	// XXX zasto je uvek 24.696?
		analogFiltered[ADC_ULAZA_koristi - 1] = cpuAdcVoltageReference;

		while (osOK != osSemaphoreAcquire(adcFinalSmf_Handle, osWaitForever)) {
			// ako istekne osWaitForever, vracam se na cekanje
			// FEA bilo sta drugo osim osWaitForever -> sta onda?
		}
		for (int i = 0; i < ADC_ULAZA_koristi; ++i) {
			float af = analogFiltered[i];
			adcAnalog[i] = af;				// finalni rezultat ADC konvertora

			if (af < 1500) {// konvertujem to isto u fejk logicki nivo (ADC rezolucija je 4096)
				adcLogic[i] = 0;		// ispod 1500 smatram da je logicka nula
			} else if (af > 2500) {
				adcLogic[i] = 1;	// preko 2500 smatram da je logicka jedinica
			};			// izmedju 1500 i 2500 stanje se ne menja. Histerezis!
		}
		if (osOK != osSemaphoreRelease(adcFinalSmf_Handle)) {
			// FEA desila se neka greska, sta sad?
		} else {
			// semafor oslobodjen, nastavi dalje
		}


	}
  /* USER CODE END analogFilterStart */
}

/* USER CODE BEGIN Header_extInt_A_receiver */
/**
* @brief Function implementing the extInt_A thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_extInt_A_receiver */
void extInt_A_receiver(void *argument)
{
  /* USER CODE BEGIN extInt_A_receiver */
	/* Infinite loop */
	for (;;) {
		if (pdTRUE == ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
			// ISR se desio, uradi nesto
			osDelay(pdMS_TO_TICKS(EXTI_MINIMUM_REPEAT_mS));		// TODO jos neko vreme ne raagujem na novi interrupt
		}
	}
  /* USER CODE END extInt_A_receiver */
}

/* USER CODE BEGIN Header_extInt_B_receiver */
/**
 * @brief Function implementing the extInt_B thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_extInt_B_receiver */
void extInt_B_receiver(void *argument)
{
  /* USER CODE BEGIN extInt_B_receiver */
	/* Infinite loop */
	for (;;) {
		if (pdTRUE == ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
			// ISR se desio, uradi nesto
			osDelay(pdMS_TO_TICKS(EXTI_MINIMUM_REPEAT_mS));		// TODO jos neko vreme ne raagujem na novi interrupt
		}
	}
  /* USER CODE END extInt_B_receiver */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void adcBackgroundStart() {
	/*
	 * ADC ulazi su: su
	 * in4
	 * in5
	 * in6
	 * in7
	 * temperature sensor (interni senzor na cpu)
	 * Vrefint (interni referentni napon za adc)
	 */

//	treptrep();
	ledon();
	osDelay(blinkiPERIOD * 2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcDMABuf, ADC_ULAZA_koristi);// pokrenem ADC konverziju

	/*
	 *
	 - ADC konverzija se zavrsi u pozadini i startuje DMA
	 - DMA prebaci rezultate u bafer adcDMABuf
	 - DMA IRQ hendler uzme semafor(1) i posalje jedan po jedan rezultat iz adc bafera u message queue
	 dma irq hendler otpusti semafor

	 - task startAnalogFilter:
	 ceka na taj queue sa osWaitforever
	 prebacuje rezultate u 2d-array istorija[sempl][ulaz]
	 izracunava srednju vrednost poslednjih X semplova, za svaki ulaz posebno
	 odredjuje logicki nivo za svaki ulaz po principu LOW < 1/3 < histerezis < 2/3 < HIGH
	 - uzme semafor(1) da prebaci rezultate u globalno vidljive array-e za analogni i logicki nivo
	 - otpusti semafor
	 - i sve se to desava u pozadini
	 -I DOK ZAVRSI, IDEM DA ODMARAM npr 10 sekundi do sledeceg starta konverzije
	 */

}

void z_HAL_ADC_DMA_irq_hendler(void) {

	HAL_ADC_Stop_DMA(&hadc1);			// zaustavi ga
	const uint8_t prioBzvz = 100;	// uvek isti prio -> nema preticanja u queue
	const uint32_t nula = 0;	// May be called from IRQ service if timeout = 0

	// ako ima slobodnih mesta bar za ceo jedan ciklus merenja
	uint32_t slob = osMessageQueueGetSpace(adcBufQue_Handle);
	if (slob >= ADC_ULAZA_koristi) {
		// saljem pojedinacna merenja u queue, ali potrebam je veliki queue
		if (osOK != osSemaphoreAcquire(adcBufSmf_Handle, nula) ) {
			// FEA desila se neka greska
		}
		for (int i = 0; i < ADC_ULAZA_koristi; ++i) {
			if (osOK != osMessageQueuePut(adcBufQue_Handle, (uint32_t*) adcDMABuf[i], prioBzvz, nula) ) {
				// FEA desila se neka greska
			}
		}
		if (osOK != osSemaphoreRelease(adcBufSmf_Handle)) {
			// FEA desila se neka greska
		}
	} else {
		// FEA nije bilo mesta u adcRezultatQueue
	}
	ledof();

}





void HAL_GPIO_EXTI_Callback(uint16_t GpioPin){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	switch (GpioPin) {
	case EXT_interrupt_A_Pin:
		vTaskNotifyGiveFromISR(extInt_AHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		break;

	case EXT_interrupt_B_Pin:
		vTaskNotifyGiveFromISR(extInt_BHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		break;

	default:
		break;
	}
}










/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
