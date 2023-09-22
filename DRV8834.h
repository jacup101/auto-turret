#ifndef DRV8834_STEPPER_DRIVER_H
#define DRV8834_STEPPER_DRIVER_H
#include <stdint.h>

#define DIR_PORT GPIOC
#define DIR_PIN GPIO_PIN_8
#define SLEEP_PORT GPIOC
#define SLEEP_PIN GPIO_PIN_9
#define STEP_PORT GPIOC
#define STEP_PIN GPIO_PIN_10

void DRV8834_init();
void DRV8834_set_dir(uint8_t dir);
void DRV8834_step();
void DRV8834_sleep();
void DRV8834_wake();

#endif
