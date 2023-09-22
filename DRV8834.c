#include "DRV8834.h"
#include "stm32f7xx_hal.h"

void DRV8834_init() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOC, DIR_PIN | SLEEP_PIN | STEP_PIN, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = DIR_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DIR_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SLEEP_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(STEP_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = STEP_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(STEP_PORT, &GPIO_InitStruct);
}

void DRV8834_set_dir(uint8_t dir) {
	if (dir == 'l') {
		HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
	} else if (dir == 'r') {
		HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
	}
}
void DRV8834_step() {
	HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
}

void DRV8834_sleep() {
	HAL_GPIO_WritePin(SLEEP_PORT, SLEEP_PIN, GPIO_PIN_RESET);
}

void DRV8834_wake() {
	HAL_GPIO_WritePin(SLEEP_PORT, SLEEP_PIN, GPIO_PIN_SET);
}





