/*!
 * @file bl0937.c
 * 
 * @brief BL0937 Driver Module
 * 
 * @author QConnex GmbH
 * @author Michael Sauer
 * 
 * @date 19. Okt 2023
 * 
 * @addtogroup sensor
 * @{
 */
 
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "bl0937.h"
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef enum bl0937_rms_output_type_e {
    BL0937_RMS_CURRENT = 0,
    BL0937_RMS_VOLTAGE = 1
} bl0937_rms_output_type_t;
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "BL0937";

volatile uint32_t bl0937_cf_frequency       = 0;
volatile uint32_t bl0937_cf1_frequency      = 0;
volatile uint32_t _bl0937_cf_frequency      = 0;
volatile uint32_t _bl0937_cf1_frequency     = 0;

static TaskHandle_t _bl0937_task_handle     = NULL;

/* Private function prototypes -----------------------------------------------*/
esp_err_t _bl0937_init_task(bl0937_config_t *config);
esp_err_t _bl0937_init_interrupt(bl0937_config_t *config);
esp_err_t _bl0937_init_gpio(bl0937_config_t *config);

/* Imported variables --------------------------------------------------------*/
/* Imported functions --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
esp_err_t bl0937_init(bl0937_config_t *config)
{
    // TODO add error handling
    _bl0937_init_gpio(config);
    _bl0937_init_task(config);
    if (_bl0937_init_interrupt(config) != ESP_OK)
        ESP_LOGE(TAG, "Error in enabling interrupts");
    return _bl0937_init_task(config);
}

void bl0937_deinit(void)
{

}

uint32_t bl0937_get_raw_cf_frequency(void)
{
    return bl0937_cf_frequency;
}

uint32_t bl0937_get_raw_cf1_frequency(void)
{
    return bl0937_cf1_frequency;
}

/* Private functions ---------------------------------------------------------*/
void IRAM_ATTR gpio_isr_handler_bl0937_cf(void* arg) {
    _bl0937_cf_frequency++;
}

void IRAM_ATTR gpio_isr_handler_bl0937_cf1(void* arg) {
    _bl0937_cf1_frequency++;
}

void _task_bl0937(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Print frequency every 1 second
        uint32_t bl0937_cf_frequency = _bl0937_cf_frequency;
        uint32_t bl0937_cf1_frequency = _bl0937_cf1_frequency;
        _bl0937_cf_frequency = 0;
        _bl0937_cf1_frequency = 0;
        printf("CF Frequency: %ld Hz, CF1 Frequency: %ld Hz\n", bl0937_cf_frequency, bl0937_cf1_frequency);
    }
}

esp_err_t _bl0937_init_task(bl0937_config_t *config)
{
    if (xTaskCreate(_task_bl0937, "bl0937", config->task_stack_size, NULL, config->task_priority, &_bl0937_task_handle) == pdTRUE) {
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
    return ESP_FAIL;
}

esp_err_t _bl0937_init_interrupt(bl0937_config_t *config)
{
    // TODO add error handling

    // check pins
    if (config->pins.cf_pin  < 0 ||
        config->pins.cf1_pin < 0 ||
        config->pins.sel_pin < 0)
        return ESP_FAIL;

    gpio_install_isr_service(0);
    gpio_isr_handler_add(config->pins.cf_pin, gpio_isr_handler_bl0937_cf, NULL);
    gpio_isr_handler_add(config->pins.cf1_pin, gpio_isr_handler_bl0937_cf1, NULL);
    return ESP_OK;
}

esp_err_t _bl0937_init_gpio(bl0937_config_t *config)
{
    // TODO add error handling

    // check pins
    if (config->pins.cf_pin  < 0 ||
        config->pins.cf1_pin < 0 ||
        config->pins.sel_pin < 0)
        return ESP_FAIL;

    // configure cf input pin
    gpio_config_t io_conf1;
    io_conf1.intr_type = GPIO_INTR_POSEDGE;
    io_conf1.pin_bit_mask = (1ULL << config->pins.cf_pin);
    io_conf1.mode = GPIO_MODE_INPUT;
    io_conf1.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf1.pull_down_en = GPIO_PULLDOWN_DISABLE;
    if (gpio_config(&io_conf1) != ESP_OK)
        return ESP_FAIL;
    
    // configure cf1 input pin
    gpio_config_t io_conf2;
    io_conf2.intr_type = GPIO_INTR_POSEDGE;
    io_conf2.pin_bit_mask = (1ULL << config->pins.cf1_pin);
    io_conf2.mode = GPIO_MODE_INPUT;
    io_conf2.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf2.pull_down_en = GPIO_PULLDOWN_DISABLE;
    if (gpio_config(&io_conf2) != ESP_OK)
        return ESP_FAIL;

    // configure sel output pin
    gpio_set_direction(config->pins.sel_pin, GPIO_MODE_OUTPUT);
    // set sel pin initially to low (current sensing)
    gpio_set_level(config->pins.sel_pin, 1);

    return ESP_OK;
}

/** @}*/

/******************* (C) COPYRIGHT 2020-2023 QConnex GmbH ***********END OF FILE****/