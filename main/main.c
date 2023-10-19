#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "bl0937.h"


#define RELAY_GPIO (1)

void app_main(void)
{
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_GPIO, 1);

    bl0937_config_t bl0937_config = BL0937_CONFIG_DEFAULT();
    bl0937_config.pins.cf_pin = 3;
    bl0937_config.pins.cf1_pin = 18;
    bl0937_config.pins.sel_pin = 5;

    bl0937_init(&bl0937_config);

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
