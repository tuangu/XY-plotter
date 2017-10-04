/*
 * ITM_write.c
 *
 *  Created on: 5.9.2016
 *      Author: krl
 */

#include <stdint.h>
#include <chip.h>
#include <usb_cdc/ITM_write.h>
// ******************************************************************
// Cortex-M SWO Trace / Debug registers used for accessing ITM
// ******************************************************************

// CoreDebug - Debug Exception and Monitor Control Register
#define DEMCR           (*((volatile uint32_t *) (0xE000EDFC)))
// DEMCR Trace Enable Bit
#define TRCENA          (1UL << 24)

// ITM Stimulus Port Access Registers
#define ITM_Port8(n)    (*((volatile uint8_t *) (0xE0000000 + 4 * n)))
#define ITM_Port16(n)   (*((volatile uint16_t *) (0xE0000000 + 4 * n)))
#define ITM_Port32(n)   (*((volatile uint32_t *) (0xE0000000 + 4 * n)))

// ITM Trace Control Register
#define ITM_TCR (*((volatile  uint32_t *) (0xE0000000 + 0xE80)))
// ITM TCR: ITM Enable bit
#define ITM_TCR_ITMENA (1UL << 0)

// ITM Trace Enable Register
#define ITM_TER (*((volatile  uint32_t *) (0xE0000000 + 0xE00)))
// ITM Stimulus Port #0 Enable bit
#define ITM_TER_PORT0ENA (1UL << 0)

void ITM_init(void) {
    /* Set up SWO to PIO1_2 */
    Chip_SWM_MovablePortPinAssign(SWM_SWO_O, 1, 2);

}

int ITM_write(const char *pcBuffer) {
    int32_t i = 0;

    // check if debugger connected and ITM channel enabled for tracing
    if ((DEMCR & TRCENA) &&
    // ITM enabled
            (ITM_TCR & ITM_TCR_ITMENA) &&
            // ITM Port #0 enabled
            (ITM_TER & ITM_TER_PORT0ENA)) {

        while (pcBuffer[i] != '\0') {
            while (ITM_Port32(0) == 0) {
            }
            ITM_Port8(0) = pcBuffer[i++];
        }
        return i;
    } else
        // Function returns zero if nothing was written
        return 0;
}

