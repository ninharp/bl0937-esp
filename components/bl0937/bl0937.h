#ifndef __BL0937_H__
#define __BL0937_H__
/*!
 * @file bl0937.h
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
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"

/* Defines -------------------------------------------------------------------*/
#define BL0937_STACK_SIZE               (4096)
#define BL0937_TASK_PRIORITY            (5)
#define BL0937_DEFAULT_COEFF_VOLTAGE    (0)
#define BL0937_DEFAULT_COEFF_CURRENT    (0)

// Maximum pulse with in microseconds
// If longer than this pulse width is reset to 0
// This value is purely experimental.
// Higher values allow for a better precission but reduce sampling rate
// and response speed to change
// Lower values increase sampling rate but reduce precission
// Values below 0.5s are not recommended since current and voltage output
// will have no time to stabilise
#ifdef CONFIG_BL0937_PULSE_TIMEOUT
#define DEFAULT_PULSE_TIME CONFIG_BL0937_PULSE_TIME
#else
#define DEFAULT_PULSE_TIMEOUT 2000000
#endif

// Internal voltage reference value
#define V_REF               2.43
//#define V_REF               1.218

// The factor of a 1mOhm resistor
// as per recomended circuit in datasheet
// A 1mOhm resistor allows a ~30A max measurement
#define R_CURRENT           0.001

// This is the factor of a voltage divider of 6x 470K upstream and 1k downstream
// as per recomended circuit in datasheet
#define R_VOLTAGE           2821

// Frequency of the BL0937 internal clock
#define F_OSC               3579000

// Minimum delay between selecting a mode and reading a sample
#define READING_INTERVAL    3000

#define BL0937_CONFIG_DEFAULT()                                 \
{                                                               \
        .task_stack_size = BL0937_STACK_SIZE,                   \
        .task_priority = BL0937_TASK_PRIORITY,                  \
        .coeff.voltage_coeff = BL0937_DEFAULT_COEFF_VOLTAGE,    \
        .coeff.current_coeff = BL0937_DEFAULT_COEFF_CURRENT,    \
        .pulse_timeout = DEFAULT_PULSE_TIMEOUT,                 \
        .pins.cf_pin = -1,                                      \
        .pins.cf1_pin = -1,                                     \
        .pins.sel_pin = -1,                                     \
        .use_interrupts = true,                                 \
        .initial_mode = BL0937_CURRENT,                         \
}

/* Exported structures -------------------------------------------------------*/
typedef struct bl0937_pins_s {
    int8_t cf_pin;
    int8_t cf1_pin;
    int8_t sel_pin;
} bl0937_pins_t;

typedef struct bl0937_coeff_s {
    uint32_t current_coeff;
    uint32_t voltage_coeff;
} bl0937_coeff_t;

typedef enum bl0937_rms_output_type_e {
    BL0937_CURRENT = 0,
    BL0937_VOLTAGE = 1
} bl0937_output_mode_t;

typedef struct bl0937_config_s {
    uint32_t task_stack_size;      //!< bl0937 task stack size
    uint32_t task_priority;        //!< bl0937 task priority
    bl0937_pins_t pins;
    bl0937_coeff_t coeff;
    uint32_t pulse_timeout;
    bool use_interrupts;
    bl0937_output_mode_t initial_mode;
} bl0937_config_t;
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
esp_err_t bl0937_init(bl0937_config_t *config);
void bl0937_deinit(void);

esp_err_t bl0937_start(void);

void bl0937_set_mode(bl0937_output_mode_t mode);
bl0937_output_mode_t bl0937_get_mode(void);
bl0937_output_mode_t bl0937_toggle_mode(void);

double bl0937_get_current(void);
unsigned int bl0937_get_voltage(void);
unsigned int bl0937_get_active_power(void);

unsigned int bl0937_get_apparent_power(void);
unsigned int bl0937_get_reactive_power(void);
double bl0937_get_power_factor(void);
unsigned long bl0937_get_energy(void);

void bl0937_reset_energy(void);

void bl0937_expected_current(double value);
void bl0937_expected_voltage(unsigned int value);
void bl0937_expected_active_power(unsigned int value);

void bl0937_reset_multipliers(void);

void bl0937_set_resistors(double current, double voltage_upstream, double voltage_downstream);

double bl0937_get_current_multiplier();
double bl0937_get_voltage_multiplier();
double bl0937_get_power_multiplier();

void bl0937_set_current_multiplier(double current_multiplier);
void bl0937_set_voltage_multiplier(double voltage_multiplier);
void bl0937_set_power_multiplier(double power_multiplier);

#ifdef _cplusplus
}
#endif

/** @}*/

#endif /* __BL0937_H__ */
/******************* (C) COPYRIGHT 2020-2023 QConnex GmbH ***********END OF FILE****/
