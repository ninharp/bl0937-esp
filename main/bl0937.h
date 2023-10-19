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


#define BL0937_CONFIG_DEFAULT()                                 \
{                                                               \
        .task_stack_size = BL0937_STACK_SIZE,                   \
        .task_priority = BL0937_TASK_PRIORITY,                  \
        .pins.cf_pin = -1,                                      \
        .pins.cf1_pin = -1,                                     \
        .pins.sel_pin = -1,                                     \
        .coeff.voltage_coeff = BL0937_DEFAULT_COEFF_VOLTAGE,    \
        .coeff.current_coeff = BL0937_DEFAULT_COEFF_CURRENT,    \
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

typedef struct bl0937_config_s {
    uint32_t task_stack_size;      //!< bl0937 task stack size
    uint32_t task_priority;        //!< bl0937 task priority
    bl0937_pins_t pins;
    bl0937_coeff_t coeff;
} bl0937_config_t;

/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
esp_err_t bl0937_init(bl0937_config_t *config);
void bl0937_deinit(void);
uint32_t bl0937_get_raw_cf_frequency(void);
uint32_t bl0937_get_raw_cf1_frequency(void);

#ifdef _cplusplus
}
#endif

/** @}*/

#endif /* __BL0937_H__ */
/******************* (C) COPYRIGHT 2020-2023 QConnex GmbH ***********END OF FILE****/
