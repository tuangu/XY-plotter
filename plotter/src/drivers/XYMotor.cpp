#include <XYMotor.h>
#include <Config.h>

#include <stdlib.h>
#include "math.h"
#include "ITM_write.h"
#include "DigitalIoPin.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

XYMotor::XYMotor(DigitalIoPin* dirX, DigitalIoPin* stepX, DigitalIoPin* dirY, DigitalIoPin *stepY,
        DigitalIoPin* lmXMin, DigitalIoPin* lmXMax, DigitalIoPin* lmYMin, DigitalIoPin* lmYMax):
    dirXPin(dirX), stepXPin(stepX), dirYPin(dirY), stepYPin(stepY),
    lmXMin(lmXMin), lmXMax(lmXMax), lmYMin(lmYMin), lmYMax(lmYMax) {

    dirXToOrigin = true;
    dirYToOrigin = false;
    sbRIT = xSemaphoreCreateBinary();

    totalStepX = 0;
    totalStepY = 0;
    isCalibrating = true;

    a = motorAccel;
    vMax = motorMaxSpeed;
    vMin = motorMinSpeed;

    accelEnd = (vMax - vMin) * (vMax - vMin) / (2.0f * a);
    sqrt_2a = sqrtf(2*a);
    microStep = 16;
}

XYMotor::~XYMotor() {
}

void XYMotor::calibrate() {
    int pps = 3600;

    dirX = dirXToOrigin;
    dirY = dirYToOrigin;

    /*
     * i = 0: move to origin
     * i = 1: 1st calibration
     * i = 2: 2nd calibration
     */
    for (int i = 0; i < 3; i++) {
        if(i == 1){
            totalStepX = 0;
            totalStepY = 0;
        }
        dirXPin->write(dirX);
        dirYPin->write(dirY);
        xState = false;
        yState = false;
        RIT_start(0, pps);  // Work around solution. Motors will stop if they hit a limit switch

        dirX = !dirX;
        dirY = !dirY;
    }

    // set calibration flag
    isCalibrating = false;

    totalStepX = totalStepX / 4;
    totalStepY = totalStepY / 4;

    currentX = 0;
    currentY = 0;

    char buffer[32];
    snprintf(buffer, 32, "X: %ld\r\nY: %ld\r\n", totalStepX, totalStepY);
    ITM_write(buffer);
}

void XYMotor::move(float toX, float toY) {
    int dx = round(xSPMM*toX) - currentX;
    int dy = round(ySPMM*toY) - currentY;

    dirX = (dx < 0) ? dirXToOrigin : !dirXToOrigin;
    dirXPin->write(dirX);
    dirY = (dy < 0) ? dirYToOrigin : !dirYToOrigin;
    dirYPin->write(dirY);

    if (abs(dx) > abs(dy)) {
        leadStep = abs(dx);
        depStep = abs(dy);

        leadStepPin = stepXPin;
        depStepPin = stepYPin;
    } else {
        leadStep = abs(dy);
        depStep = abs(dx);

        leadStepPin = stepYPin;
        depStepPin = stepXPin;
    }

    delta = 2 * depStep - leadStep; 

//    float dv = vMax - vMin;
//    float accelEnd = dv * dv / (2.0f * a); // length of the acceleration phase, [step]


//    int accelEnd = fmin(aStep, leadStep / 2);
//    int decelStart = leadStep - accelEnd + 1;

    currentDepStep = 0;
    currentLeadStep = 0;

    while (currentLeadStep < leadStep) {
        if (currentLeadStep < accelEnd) {           // accelerating
            float v = sqrt_2a * sqrtf(currentLeadStep) + vMin;
            int pps = v * microStep;
            RIT_start(2, pps);
//        } else if (currentLeadStep < decelStart) {  // constant speed
//            float v = sqrt_2a * sqrtf(accelEnd) + vMin;
//            int pps = v * microStep;
//            RIT_start(2 * (decelStart - accelEnd + 1), pps);
//        } else if (currentLeadStep < leadStep) {                  // decelerating
//            float v = sqrt_2a * sqrtf(leadStep - currentLeadStep - 1);
//            int pps = v * microStep;
//            RIT_start(2, pps);
//        }
        } else {
            float v = sqrt_2a * sqrtf(accelEnd) + vMin;
            int pps = v * microStep;
            RIT_start(2 * (leadStep - accelEnd + 1), pps);
        }
    }

    currentX = round(xSPMM*toX);
    currentY = round(ySPMM*toY);
}

bool XYMotor::irqHandler() {
    portBASE_TYPE xHigherPriorityWoken = pdFALSE;

    Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag

    bool minX = lmXMin->read() && (dirX == dirXToOrigin);
    bool maxX = lmXMax->read() && (dirX == !dirXToOrigin);
    bool minY = lmYMin->read() && (dirY == dirYToOrigin);
    bool maxY = lmYMax->read() && (dirY == !dirYToOrigin);

    if (minX || maxX || minY || maxY) {
        // motors hit limit switches, stop
        Chip_RIT_Disable(LPC_RITIMER);
        xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
    }

    if ((RIT_count > 0) && ((RIT_count % 2) == 0)) {          // write LOW to step pin
        RIT_count--;
        leadStepPin->write(0);

        if (delta > 0) {
            depStepPin->write(0);
        }
    } else if ((RIT_count > 0) && ((RIT_count % 2) != 0)) {   // write HIGH to step pin
        RIT_count--;
        currentLeadStep += 1;
        leadStepPin->write(1);

        if (delta > 0) {
            currentDepStep += 1;
            depStepPin->write(1);

            delta = delta - 2 * leadStep;
        }

        delta = delta + 2 * depStep;
    } else {
        Chip_RIT_Disable(LPC_RITIMER);                      // disable timer
        xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);// give semaphore
    }

    return xHigherPriorityWoken;
}

bool XYMotor::irqHandlerCalibration() {
    portBASE_TYPE xHigherPriorityWoken = pdFALSE;

    Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag

    bool minX = lmXMin->read() && (dirX == dirXToOrigin);
    bool maxX = lmXMax->read() && (dirX == !dirXToOrigin);
    bool minY = lmYMin->read() && (dirY == dirYToOrigin);
    bool maxY = lmYMax->read() && (dirY == !dirYToOrigin);

    if (!minX && !maxX) {
        stepXPin->write(xState);
        xState = !xState;
        totalStepX++;
    }

    if (!minY && !maxY) {
        stepYPin->write(yState);
        yState = !yState;
        totalStepY++;
    }

    if ((minX || maxX) && (minY || maxY)) {
        // disable timer
        Chip_RIT_Disable(LPC_RITIMER);

        // give semaphore and set context switch flag if a higher priority task was woken up
        xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
    }

    return xHigherPriorityWoken;
}

void XYMotor::RIT_start(int count, int pps) {
    uint64_t cmp_value;

    // Determine approximate compare value based on clock rate and passed interval
    cmp_value = (uint64_t) Chip_Clock_GetSystemClockRate() / pps;

    // disable timer during configuration
    Chip_RIT_Disable(LPC_RITIMER);

    RIT_count = count;

    // enable automatic clear on when compare value==timer value
    // this makes interrupts trigger periodically
    Chip_RIT_EnableCompClear(LPC_RITIMER);

    // reset the counter
    Chip_RIT_SetCounter(LPC_RITIMER, 0);
    Chip_RIT_SetCompareValue(LPC_RITIMER, cmp_value);

    // start counting
    Chip_RIT_Enable(LPC_RITIMER);

    // Enable the interrupt signal in NVIC (the interrupt controller)
    NVIC_EnableIRQ(RITIMER_IRQn);

    // wait for ISR to tell that we're done
    if (xSemaphoreTake(sbRIT, portMAX_DELAY) == pdTRUE) {
        // Disable the interrupt signal in NVIC (the interrupt controller)
        NVIC_DisableIRQ(RITIMER_IRQn);
    } else {
        // unexpected error
    }
}
