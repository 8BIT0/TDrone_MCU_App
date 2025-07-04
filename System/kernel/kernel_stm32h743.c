/*
 * coder: 8_B!T0
 * Kernel Funciont portable for any Kernel u use
 * for stm32 some assemble and regsiter are in common
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../../FCHW_Config.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_pwr.h"


TIM_HandleTypeDef htim17;
TIM_HandleTypeDef htim16;
bool Kernel_TickTimer_Init = false;

static bool KernelClock_Init(void);
bool Kernel_BaseTick_Init(void);
bool Kernel_Sys_BaseTick_Init(void);
void Kernel_MPU_Config(void);

bool Kernel_Init(void)
{
	__enable_irq();

    Kernel_MPU_Config();
    SCB_EnableICache();
    SCB_EnableDCache();

    HAL_Init();

    return KernelClock_Init() & Kernel_BaseTick_Init() & Kernel_Sys_BaseTick_Init();
}

void Kernel_MPU_Config(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct = {0};

	HAL_MPU_Disable();
	MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
	MPU_InitStruct.BaseAddress      = (uint32_t)0xC0000000;
	MPU_InitStruct.Size             = MPU_REGION_SIZE_32MB;
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
	MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
	MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
	MPU_InitStruct.SubRegionDisable = 0x00;
	MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/*
 * clock init
 * general by stm32cubemx
 */
static bool KernelClock_Init(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	__HAL_RCC_GPIOH_CLK_ENABLE();
	
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 128;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		return false;

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
								|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        return false;

	PeriphClkInitStruct.PLL2.PLL2M = 5;
	PeriphClkInitStruct.PLL2.PLL2N = 128;
	PeriphClkInitStruct.PLL2.PLL2P = 4;
	PeriphClkInitStruct.PLL2.PLL2Q = 4;
	PeriphClkInitStruct.PLL2.PLL2R = 4;
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
		return false;

    return true;
}

bool Kernel_Sys_BaseTick_Init(void)
{
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 15;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 9980; /* 9980 tick unit represent 1Ms, 1tick value represent 1000us / 9980 */
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
		return false;

	if(HAL_TIM_Base_Start_IT(&htim16) != HAL_OK)
		return false;
	
	Kernel_TickTimer_Init = true; 
	
	return true;
}

bool Kernel_EnableTimer_IRQ(void)
{
	if(Kernel_TickTimer_Init)
	{
		if(HAL_TIM_Base_Start_IT(&htim16) != HAL_OK)
			return false;
	
		return true;
	}

	return false;
}

bool Kernel_DisableTimer_IRQ(void)
{
	if(Kernel_TickTimer_Init)
	{
		if(HAL_TIM_Base_Stop_IT(&htim16) != HAL_OK)
			return false;
	
		return true;
	}

	return false;
}

uint32_t Kernel_TickVal_To_Us(void)
{
	if(Kernel_TickTimer_Init)
		return __HAL_TIM_GET_AUTORELOAD(&htim16) / 1000;

	return 0;
}

uint32_t Kernel_Get_PeriodValue(void)
{
	if(Kernel_TickTimer_Init)
		return __HAL_TIM_GET_AUTORELOAD(&htim16);

	return 0;
}

bool Kernel_Set_PeriodValue(uint32_t value)
{
	int32_t set_diff = value;
	uint32_t cur_systick_period = 0;

	if(Kernel_TickTimer_Init)
	{
		/* sys default tick Period count is 10000 */
		/* 10000tick unit represent 1Ms, 1tick unit represent 100Ns */
		cur_systick_period = __HAL_TIM_GET_AUTORELOAD(&htim16);
		set_diff -= cur_systick_period;

		/* single tune range 10Us to -10Us */
		if((set_diff <= 100) || (set_diff >= -100))
		{
			__HAL_TIM_SET_AUTORELOAD(&htim16, value);
		}
		else if(set_diff > 100)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim16, cur_systick_period + 100);
		}
		else if(set_diff < -100)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim16, cur_systick_period - 100);
		}

		return true;
	}

	return false;
}

uint32_t Kernel_Get_SysTimer_TickUnit(void)
{
	if(Kernel_TickTimer_Init)
		return __HAL_TIM_GET_COUNTER(&htim16);

	return 0;
}

bool Kernel_Set_SysTimer_TickUnit(uint32_t unit)
{
	uint32_t addin = unit;

	if(Kernel_TickTimer_Init)
	{
		if(addin > __HAL_TIM_GET_AUTORELOAD(&htim16))
			addin = __HAL_TIM_GET_AUTORELOAD(&htim16);

		__HAL_TIM_SET_COUNTER(&htim16, addin);
		return true;
	}

	return false;
}

void Kernel_BaseTick_DeInit(void)
{
    HAL_TIM_Base_Stop_IT(&htim16);
    HAL_TIM_Base_Stop_IT(&htim17);

	HAL_TIM_Base_DeInit(&htim16);
	HAL_TIM_Base_DeInit(&htim17);
	HAL_NVIC_DisableIRQ(PendSV_IRQn);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
	if (htim_base->Instance == TIM16)
	{
		/* Peripheral clock enable */
		__HAL_RCC_TIM16_CLK_ENABLE();

		/* TIM17 interrupt Init */
		HAL_NVIC_SetPriority(TIM16_IRQn, 14, 0);
		HAL_NVIC_EnableIRQ(TIM16_IRQn);
	}
	else if (htim_base->Instance == TIM17)
	{
		/* Peripheral clock enable */
		__HAL_RCC_TIM17_CLK_ENABLE();

		/* TIM17 interrupt Init */
		HAL_NVIC_SetPriority(TIM17_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(TIM17_IRQn);
	}
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
    if (htim_base->Instance == TIM16)
    {
        __HAL_RCC_TIM16_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM16_IRQn);
    }
    else if (htim_base->Instance == TIM17)
    {
        __HAL_RCC_TIM17_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM17_IRQn);
    }
}

bool Kernel_BaseTick_Init(void)
{
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 0;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 160000;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
		return false;

	if(HAL_TIM_Base_Start_IT(&htim17) != HAL_OK)
		return false;

	return true;
}

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
	__HAL_RCC_SYSCFG_CLK_ENABLE();

	/* System interrupt init*/
	/* PendSV_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}

void Kernel_reboot(void)
{
	__set_FAULTMASK(1);

	NVIC_SystemReset();
}
