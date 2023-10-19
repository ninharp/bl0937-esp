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
#include <math.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "bl0937.h"
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const char *TAG = "BL0937";

int8_t _bl0937_cf_pin;
int8_t _bl0937_cf1_pin;
int8_t _bl0937_sel_pin;

double _bl0937_current_resistor = R_CURRENT;
double _bl0937_voltage_resistor = R_VOLTAGE;

double _bl0937_current_multiplier;                                          // Unit: us/A
double _bl0937_voltage_multiplier;                                          // Unit: us/V
double _bl0937_power_multiplier;                                            // Unit: us/W

uint32_t _bl0937_pulse_timeout                  = DEFAULT_PULSE_TIMEOUT;    //Unit: us
volatile uint32_t _bl0937_voltage_pulse_width   = 0;                        //Unit: us
volatile uint32_t _bl0937_current_pulse_width   = 0;                        //Unit: us
volatile uint32_t _bl0937_power_pulse_width     = 0;                        //Unit: us
volatile uint32_t _bl0937_pulse_count           = 0;

double _bl0937_current                          = 0;
unsigned int _bl0937_voltage                    = 0;
unsigned int _bl0937_power                      = 0;

unsigned char _bl0937_current_mode              = 1;
volatile unsigned char _bl0937_mode;

bool _bl0937_use_interrupts                     = true;
volatile uint32_t _bl0937_last_cf_interrupt     = 0;
volatile uint32_t _bl0937_last_cf1_interrupt    = 0;
volatile uint32_t _bl0937_first_cf1_interrupt   = 0;

static bool _bl0937_is_initialized              = false;

static uint32_t _bl0937_task_stack_size;
static uint32_t _bl0937_task_priority;

/* Private function prototypes -----------------------------------------------*/
uint32_t _bl0937_get_pulse_in(uint32_t pin, uint32_t level, uint32_t timeout_us);

esp_err_t _bl0937_init_interrupt(void);
esp_err_t _bl0937_init_gpio(void);

void _bl0937_check_cf_signal();
void _bl0937_check_cf1_signal();
void _bl0937_calculate_default_multipliers();

/* Imported variables --------------------------------------------------------*/
/* Imported functions --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
esp_err_t bl0937_init(bl0937_config_t *config)
{
    _bl0937_current_mode    = config->initial_mode;
    _bl0937_pulse_timeout   = config->pulse_timeout;
    _bl0937_use_interrupts  = config->use_interrupts;
    _bl0937_task_stack_size = config->task_stack_size;
    _bl0937_task_priority   = config->task_priority;

    // check pins
    if (config->pins.cf_pin  < 0 ||
        config->pins.cf1_pin < 0 ||
        config->pins.sel_pin < 0) {
        ESP_LOGE(TAG, "Pin configuration invalid!");
        return ESP_FAIL;
    }

    _bl0937_cf_pin          = config->pins.cf_pin;
    _bl0937_cf1_pin         = config->pins.cf1_pin;
    _bl0937_sel_pin         = config->pins.sel_pin;


    if (_bl0937_init_gpio() != ESP_OK) {
        ESP_LOGE(TAG, "Could not initalize GPIO!");
        return ESP_FAIL;
    }

    _bl0937_is_initialized = true;

    return ESP_OK;
}

void bl0937_deinit(void)
{
    if (!_bl0937_is_initialized)
        return;

    if (_bl0937_use_interrupts) {
        // gpio_uninstall_isr_service();
        if (gpio_isr_handler_remove(_bl0937_cf_pin) == ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "IRQs not initialized for deinit!");
            return;
        }
        gpio_isr_handler_remove(_bl0937_cf1_pin);
    }
}

esp_err_t bl0937_start(void)
{
    if (_bl0937_is_initialized && _bl0937_use_interrupts) {
        if (_bl0937_init_interrupt() != ESP_OK) {
            ESP_LOGE(TAG, "Error in enabling interrupts");
            return ESP_FAIL;
        }
        return ESP_OK;
    } else if (_bl0937_is_initialized && !_bl0937_use_interrupts) {
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "BL0937 is not initalized!");
        return ESP_FAIL;
    }
}

void bl0937_set_mode(bl0937_output_mode_t mode)
{
    _bl0937_mode = (mode == BL0937_CURRENT) ? _bl0937_current_mode : 1 - _bl0937_current_mode;
    gpio_set_level(_bl0937_sel_pin, _bl0937_mode);
    if (_bl0937_use_interrupts) {
        _bl0937_last_cf1_interrupt = _bl0937_first_cf1_interrupt = esp_timer_get_time();
    }
}

bl0937_output_mode_t bl0937_get_mode(void)
{
    return (_bl0937_mode == _bl0937_current_mode) ? BL0937_CURRENT : BL0937_VOLTAGE;
}

bl0937_output_mode_t bl0937_toggle_mode(void)
{
    bl0937_output_mode_t new_mode = bl0937_get_mode() == BL0937_CURRENT ? BL0937_VOLTAGE : BL0937_CURRENT;
    bl0937_set_mode(new_mode);
    return new_mode;
}

double bl0937_get_current(void)
{
    // Power measurements are more sensitive to switch offs,
    // so we first check if power is 0 to set _bl0937_current to 0 too
    if (_bl0937_power == 0) {
        _bl0937_current_pulse_width = 0;

    } else if (_bl0937_use_interrupts) {
        _bl0937_check_cf1_signal();
    } else if (_bl0937_mode == _bl0937_current_mode) {
        _bl0937_current_pulse_width = _bl0937_get_pulse_in(_bl0937_cf1_pin, 1, _bl0937_pulse_timeout);
    }

    _bl0937_current = (_bl0937_current_pulse_width > 0) ? _bl0937_current_multiplier / _bl0937_current_pulse_width / 2 : 0;
    return _bl0937_current;

}

unsigned int bl0937_get_voltage(void)
{
    if (_bl0937_use_interrupts) {
        _bl0937_check_cf1_signal();
    } else if (_bl0937_mode != _bl0937_current_mode) {
        _bl0937_voltage_pulse_width = _bl0937_get_pulse_in(_bl0937_cf1_pin, 1, _bl0937_pulse_timeout);
    }
    _bl0937_voltage = (_bl0937_voltage_pulse_width > 0) ? _bl0937_voltage_multiplier / _bl0937_voltage_pulse_width / 2 : 0;
    return _bl0937_voltage;
}

unsigned int bl0937_get_active_power(void)
{
    if (_bl0937_use_interrupts) {
        _bl0937_check_cf_signal();
    } else {
        _bl0937_power_pulse_width = _bl0937_get_pulse_in(_bl0937_cf_pin, 1, _bl0937_pulse_timeout);
    }
    _bl0937_power = (_bl0937_power_pulse_width > 0) ? _bl0937_power_multiplier / _bl0937_power_pulse_width / 2 : 0;
    return _bl0937_power;
}

unsigned int bl0937_get_apparent_power(void)
{
    double current = bl0937_get_current();
    unsigned int voltage = bl0937_get_voltage();
    return voltage * current;
}

unsigned int bl0937_get_reactive_power(void)
{
    unsigned int active = bl0937_get_active_power();
    unsigned int apparent = bl0937_get_apparent_power();
    if (apparent > active) {
        return sqrt(apparent * apparent - active * active);
    } else {
        return 0;
    }
}

double bl0937_get_power_factor(void)
{
    unsigned int active = bl0937_get_active_power();
    unsigned int apparent = bl0937_get_apparent_power();
    if (active > apparent) return 1;
    if (apparent == 0) return 0;
    return (double) active / apparent;
}

unsigned long bl0937_get_energy(void)
{

    // Counting pulses only works in IRQ mode
    if (!_bl0937_use_interrupts)
        return 0;

    /*
        Pulse count is directly proportional to energy:
        P = m*f (m=power multiplier, f = Frequency)
        f = N/t (N=pulse count, t = time)
        E = P*t = m*N  (E=energy)
    */
    return _bl0937_pulse_count * _bl0937_power_multiplier / 1000000. / 2;

}

void bl0937_reset_energy(void)
{
    _bl0937_pulse_count = 0;
}

void bl0937_expected_current(double value)
{
    if (_bl0937_current == 0) bl0937_get_current();
    if (_bl0937_current > 0) _bl0937_current_multiplier *= (value / _bl0937_current);
}

void bl0937_expected_voltage(unsigned int value)
{
    if (_bl0937_voltage == 0) bl0937_get_voltage();
    if (_bl0937_voltage > 0) _bl0937_voltage_multiplier *= ((double) value / _bl0937_voltage);
}

void bl0937_expected_active_power(unsigned int value)
{
    if (_bl0937_power == 0) bl0937_get_active_power();
    if (_bl0937_power > 0) _bl0937_power_multiplier *= ((double) value / _bl0937_power);
}

void bl0937_reset_multipliers(void)
{
    _bl0937_calculate_default_multipliers();
}

void bl0937_set_resistors(double current, double voltage_upstream, double voltage_downstream)
{
    if (voltage_downstream > 0) {
        _bl0937_current_resistor = current;
        _bl0937_voltage_resistor = (voltage_upstream + voltage_downstream) / voltage_downstream;
        _bl0937_calculate_default_multipliers();
    }
}

double bl0937_get_current_multiplier() { return _bl0937_current_multiplier; };
double bl0937_get_voltage_multiplier() { return _bl0937_voltage_multiplier; };
double bl0937_get_power_multiplier() { return _bl0937_power_multiplier; };

void bl0937_set_current_multiplier(double current_multiplier) { _bl0937_current_multiplier = current_multiplier; };
void bl0937_set_voltage_multiplier(double voltage_multiplier) { _bl0937_voltage_multiplier = voltage_multiplier; };
void bl0937_set_power_multiplier(double power_multiplier) { _bl0937_power_multiplier = power_multiplier; };

/* Private functions ---------------------------------------------------------*/
void IRAM_ATTR gpio_isr_handler_bl0937_cf(void* arg)
{
    unsigned long now = esp_timer_get_time();
    _bl0937_power_pulse_width = now - _bl0937_last_cf_interrupt;
    _bl0937_last_cf_interrupt = now;
    _bl0937_pulse_count++;
}

void IRAM_ATTR gpio_isr_handler_bl0937_cf1(void* arg)
{
    unsigned long now = esp_timer_get_time();

    if ((now - _bl0937_first_cf1_interrupt) > _bl0937_pulse_timeout) {

        unsigned long pulse_width;
        
        if (_bl0937_last_cf1_interrupt == _bl0937_first_cf1_interrupt) {
            pulse_width = 0;
        } else {
            pulse_width = now - _bl0937_last_cf1_interrupt;
        }

        if (_bl0937_mode == _bl0937_current_mode) {
            _bl0937_current_pulse_width = pulse_width;
        } else {
            _bl0937_voltage_pulse_width = pulse_width;
        }

        _bl0937_mode = 1 - _bl0937_mode;
        gpio_set_level(_bl0937_sel_pin, _bl0937_mode);
        _bl0937_first_cf1_interrupt = now;

    }

    _bl0937_last_cf1_interrupt = now;
}

uint32_t _bl0937_get_pulse_in(uint32_t pin, uint32_t level, uint32_t timeout_us)
{
    uint32_t pulse_duration = 0;
    TickType_t start_time = xTaskGetTickCount();
    bool pulse_started = false;

    while (gpio_get_level(pin) != level) {
        if (xTaskGetTickCount() - start_time >= pdMS_TO_TICKS(timeout_us / 1000)) {
            return 0;  // Timeout
        }
    }

    while (1) {
        if (gpio_get_level(pin) == level) {
            if (!pulse_started) {
                start_time = xTaskGetTickCount();
                pulse_started = true;
            }
        } else if (pulse_started) {
            pulse_duration = (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS;
            break;
        }
    }

    return pulse_duration;
}

esp_err_t _bl0937_init_interrupt(void)
{
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK) {
        ESP_LOGE("Could not install ISR service!");
        return err;
    }

    err = gpio_isr_handler_add(_bl0937_cf_pin, gpio_isr_handler_bl0937_cf, NULL);
    if (err == ESP_ERR_INVALID_ARG)
        ESP_LOGE(TAG, "Pin configuration for CF pin is invalid!");
    else
        ESP_LOGE(TAG, "Error in adding ISR handler for CF pin");

    err = gpio_isr_handler_add(_bl0937_cf1_pin, gpio_isr_handler_bl0937_cf1, NULL);
    if (err == ESP_ERR_INVALID_ARG)
        ESP_LOGE(TAG, "Pin configuration for CF1 pin is invalid!");
    else
        ESP_LOGE(TAG, "Error in adding ISR handler for CF1 pin");

    return err;
}

esp_err_t _bl0937_init_gpio(void)
{
    // configure cf input pin
    gpio_config_t io_conf1;
    io_conf1.intr_type = GPIO_INTR_POSEDGE;
    io_conf1.pin_bit_mask = (1ULL << _bl0937_cf_pin);
    io_conf1.mode = GPIO_MODE_INPUT;
    io_conf1.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf1.pull_down_en = GPIO_PULLDOWN_DISABLE;
    if (gpio_config(&io_conf1) != ESP_OK)
        return ESP_FAIL;
    
    // configure cf1 input pin
    gpio_config_t io_conf2;
    io_conf2.intr_type = GPIO_INTR_POSEDGE;
    io_conf2.pin_bit_mask = (1ULL << _bl0937_cf1_pin);
    io_conf2.mode = GPIO_MODE_INPUT;
    io_conf2.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf2.pull_down_en = GPIO_PULLDOWN_DISABLE;
    if (gpio_config(&io_conf2) != ESP_OK)
        return ESP_FAIL;

    // configure sel output pin
    gpio_set_direction(_bl0937_sel_pin, GPIO_MODE_OUTPUT);
    // set sel pin initially to low (current sensing)
    gpio_set_level(_bl0937_sel_pin, 1);

    return ESP_OK;
}

void _bl0937_check_cf_signal(void)
{
    if ((esp_timer_get_time() - _bl0937_last_cf_interrupt) > _bl0937_pulse_timeout) _bl0937_power_pulse_width = 0;
}

void _bl0937_check_cf1_signal()
{
    if ((esp_timer_get_time() - _bl0937_last_cf1_interrupt) > _bl0937_pulse_timeout) {
        if (_bl0937_mode == _bl0937_current_mode) {
            _bl0937_current_pulse_width = 0;
        } else {
            _bl0937_voltage_pulse_width = 0;
        }
        bl0937_toggle_mode();
    }
}

void _bl0937_calculate_default_multipliers(void)
{
     _bl0937_current_multiplier = ( 1000000.0 * 512 * V_REF / _bl0937_current_resistor / 24.0 / F_OSC );
     _bl0937_voltage_multiplier = ( 1000000.0 * 512 * V_REF * _bl0937_voltage_resistor / 2.0 / F_OSC );
     _bl0937_power_multiplier = ( 1000000.0 * 128 * V_REF * V_REF * _bl0937_voltage_resistor / _bl0937_current_resistor / 48.0 / F_OSC );
}

/** @}*/

/******************* (C) COPYRIGHT 2020-2023 QConnex GmbH ***********END OF FILE****/