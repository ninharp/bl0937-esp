#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_timer.h"

#include "bl0937.h"


// GPIOs
#define RELAY_PIN                       (1)
#define SEL_PIN                         (5)
#define CF1_PIN                         (18)
#define CF_PIN                          (3)

// Check values every 10 seconds
#define UPDATE_TIME                     5000

// Set SEL_PIN to HIGH to sample current
// This is the case for Itead's Sonoff POW, where a
// the SEL_PIN drives a transistor that pulls down
// the SEL pin in the BL0937 when closed
#define CURRENT_MODE                    1

// These are the nominal values for the resistors in the circuit
#define CURRENT_RESISTOR                0.001
#define VOLTAGE_RESISTOR_UPSTREAM       ( 5 * 470000 ) // Real: 2280k
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 ) // Real 1.009k

static const char *TAG = "main";

void calibrate(void)
{
    // Let some time to register values
    unsigned long timeout = esp_timer_get_time() / 1000;
    while (((esp_timer_get_time() / 1000) - timeout) < 10000) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Calibrate using a 50W bulb (pure resistive) on a 230V line
    bl0937_expected_active_power(50.0);
    bl0937_expected_voltage(230.0);
    bl0937_expected_current(50.0 / 230.0);

    // Show corrected factors
    ESP_LOGI(TAG, "New current multiplier : %.2f", bl0937_get_current_multiplier());
    ESP_LOGI(TAG, "New voltage multiplier : %.2f", bl0937_get_voltage_multiplier());
    ESP_LOGI(TAG, "New power multiplier   : %.2f", bl0937_get_power_multiplier());
}

void app_main(void)
{
    gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_PIN, 1);

    bl0937_config_t bl0937_config   = BL0937_CONFIG_DEFAULT();
    bl0937_config.pins.cf_pin       = CF_PIN;
    bl0937_config.pins.cf1_pin      = CF1_PIN;
    bl0937_config.pins.sel_pin      = SEL_PIN;
    bl0937_config.initial_mode      = CURRENT_MODE;

    bl0937_init(&bl0937_config);

    // These values are used to calculate current, voltage and power factors as per datasheet formula
    // These are the nominal values for the Sonoff POW resistors:
    // * The CURRENT_RESISTOR is the 1milliOhm copper-manganese resistor in series with the main line
    // * The VOLTAGE_RESISTOR_UPSTREAM are the 5 470kOhm resistors in the voltage divider that feeds the V2P pin in the BL0937
    // * The VOLTAGE_RESISTOR_DOWNSTREAM is the 1kOhm resistor in the voltage divider that feeds the V2P pin in the BL0937
    bl0937_set_resistors(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);

    // Show default (as per datasheet) multipliers
    ESP_LOGI(TAG, "Default current multiplier : %.2f", bl0937_get_current_multiplier());
    ESP_LOGI(TAG, "Default voltage multiplier : %.2f", bl0937_get_voltage_multiplier());
    ESP_LOGI(TAG, "Default power multiplier   : %.2f", bl0937_get_power_multiplier());

    bl0937_start();

    calibrate();

    unsigned long last = (esp_timer_get_time() / 1000);

    while (1) {

        // This UPDATE_TIME should be at least twice the interrupt timeout (2 second by default)
        if (((esp_timer_get_time() / 1000) - last) > UPDATE_TIME) {

            last = (esp_timer_get_time() / 1000);
            ESP_LOGI(TAG, "Active Power (W)    : %d",   bl0937_get_active_power());
            ESP_LOGI(TAG, "Voltage (V)         : %d",   bl0937_get_voltage());
            ESP_LOGI(TAG, "Current (A)         : %.2f", bl0937_get_current());
            ESP_LOGI(TAG, "Apparent Power (VA) : %d",   bl0937_get_apparent_power());
            ESP_LOGI(TAG, "Power Factor (%%)   : %d",   (int) (100 * bl0937_get_power_factor()));
            ESP_LOGI(TAG, "Agg. energy (Ws)    : %ld",  bl0937_get_energy());
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Print frequency every 1 second
    }

}
