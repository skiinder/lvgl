/**
 * @file lv_aoostar.h
 *
 * AOOSTAR GEM12 / WTR MAX secondary screen display driver
 * 960x376 RGB565 over USB CDC ACM @ 1.5Mbps
 *
 * Protocol: AA 55 AA 55 + cmd + offset(LE) + data
 * Chunk size: 48 bytes (aligns with row: 40 chunks/row)
 * Transfer: DMA-friendly - all changed chunks encoded into one buffer,
 *           sent via single write().
 */

#ifndef LV_AOOSTAR_H
#define LV_AOOSTAR_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "../../../display/lv_display.h"

#if LV_USE_AOOSTAR

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create an AOOSTAR display driver.
 * Automatically detects /dev/ttyACM0 by USB VID:PID (0416:90a1).
 * @return  pointer to the created display, or NULL on error.
 */
lv_display_t * lv_aoostar_create(void);

/**
 * Set a specific serial device path instead of auto-detect.
 * @param disp      pointer to an AOOSTAR display
 * @param device    serial device path (e.g. "/dev/ttyACM1")
 * @return          LV_RESULT_OK on success
 */
lv_result_t lv_aoostar_set_device(lv_display_t * disp, const char * device);

/**********************
 *      MACROS
 **********************/

#endif /* LV_USE_AOOSTAR */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LV_AOOSTAR_H */
