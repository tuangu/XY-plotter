#include <XYMotor.h>

#include <stdlib.h>
#include "ITM_write.h"
#include "DigitalIoPin.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

XYMotor::XYMotor(DigitalIoPin* dirX, DigitalIoPin* stepX, DigitalIoPin* dirY, DigitalIoPin *stepY,
        DigitalIoPin* lmXMin, DigitalIoPin* lmXMax, DigitalIoPin* lmYMin, DigitalIoPin* lmYMax):
    dirXPin(dirX), stepXPin(stepX), dirYPin(dirY), stepYPin(stepY),
    lmXMin(lmXMin), lmXMax(lmXMax), lmYMin(lmYMin), lmYMax(lmYMax) {

    dirToOrigin = false; // true;
    sbRIT = xSemaphoreCreateBinary();

    totalStepX = 0;
    totalStepY = 0;
    isCalibrating = true;
}

XYMotor::~XYMotor() {

}

void XYMotor::calibrate() {
    int pps = 1600;

    dirX = dirToOrigin;
    dirY = dirToOrigin;
    totalStepX = 0;
    totalStepY = 0;
    /*
     * i = 0: move to origin
     * i = 1: 1st calibration
     * i = 2: 2nd calibration
     */
    for (int i = 0; i < 3; i++) {
        dirXPin->write(dirX);
        dirYPin->write(dirY);
        xState = false;
        yState = false;
        RIT_start(pps);

        dirX = !dirX;
        dirY = !dirY;
    }

    // set calibration flag
    isCalibrating = false;

    totalStepX = totalStepX / 4 + 1;
    totalStepY = totalStepY / 4 + 1;

    char buffer[32];
    snprintf(buffer, 32, "X: %ld\r\nY: %ld\r\n", totalStepX, totalStepY);
    ITM_write(buffer);
}

void XYMotor::move(float fromX, float fromY, float toX, float toY, int pps) {
    float dx = toX - fromX;
    float dy = toY - fromY;

    dirX = (dx < 0) ? dirToOrigin : !dirToOrigin;
    dirXPin->write(dirX);
    dirY = (dy < 0) ? dirToOrigin : !dirToOrigin;
    dirYPin->write(dirY);

    x = 0; xState = false;
    y = 0; yState = false;

    stepX = abs(dx * totalStepX / baseX);
    stepY = abs(dy * totalStepY / baseY);

    if (stepX < stepY) {
        stepX = abs(dy * totalStepY / baseY);
        stepY = abs(dx * totalStepX / baseX);

        tempXPin = stepYPin;
        tempYPin = stepXPin;
    } else {
        tempXPin = stepXPin;
        tempYPin = stepYPin;
    }

    delta = 2 * stepY - stepX;
    motorYMove = (delta > 0) ? true : false;

    isUpdateDelta = false;

    RIT_start(pps);
}

bool XYMotor::irqHandler() {
    portBASE_TYPE xHigherPriorityWoken = pdFALSE;

    Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag

    if(x == stepX && y == stepY) {
        // disable timer
        Chip_RIT_Disable(LPC_RITIMER);

        // give semaphore and set context switch flag if a higher priority task was woken up
        xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
    } else {

        // move motor X, Y
        // need 2 interrupts to drive step pin from LOW to HIGH
        if (x < stepX) {
            tempXPin->write(xState);
            x += (xState) ? 1 : 0;
            xState = !xState;
        }

        if (motorYMove && (y < stepY)) {
            tempYPin->write(yState);
            if (yState)
                motorYMove = false;
            yState = !yState;
        }

        // update delta every 2 interrupts
        if (isUpdateDelta) {
            if (delta > 0) {
                y += 1;
                delta = delta - 2 * stepX;
                motorYMove = true;
            }
            delta = delta + 2 * stepY;
        }

        isUpdateDelta = !isUpdateDelta;

        // check limit switches
        bool minX = lmXMin->read() && (dirX == dirToOrigin);
        bool maxX = lmXMax->read() && (dirX == !dirToOrigin);
        bool minY = lmYMin->read() && (dirY == dirToOrigin);
        bool maxY = lmYMax->read() && (dirY == !dirToOrigin);

        if (minX || maxX)
            x = stepX; // X motor will stop in the next interrupt
        if (minY || maxY)
            y = stepY; // Y motor will stop in the next interrupt
    }

    return xHigherPriorityWoken;
}

bool XYMotor::irqHandlerCalibration() {
    portBASE_TYPE xHigherPriorityWoken = pdFALSE;

    Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag

    bool minX = lmXMin->read() && (dirX == dirToOrigin);
    bool maxX = lmXMax->read() && (dirX == !dirToOrigin);
    bool minY = lmYMin->read() && (dirY == dirToOrigin);
    bool maxY = lmYMax->read() && (dirY == !dirToOrigin);

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

void XYMotor::RIT_start(int pps) {
    uint64_t cmp_value;

    // Determine approximate compare value based on clock rate and passed interval
    cmp_value = (uint64_t) Chip_Clock_GetSystemClockRate() / pps;

    // disable timer during configuration
    Chip_RIT_Disable(LPC_RITIMER);

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

void XYMotor::setBaseX(int base) {
    baseX = base;
}

void XYMotor::setBaseY(int base) {
    baseY = base;
}

void XYMotor::setTotalStepX(long total) {
    totalStepX = total;
}

void XYMotor::setTotalStepY(long total) {
    totalStepY = total;
}
