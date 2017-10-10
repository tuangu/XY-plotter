#include <XYMotor.h>

#include <stdlib.h>
#include "DigitalIoPin.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

XYMotor::XYMotor(DigitalIoPin* dirX, DigitalIoPin* stepX, DigitalIoPin* dirY, DigitalIoPin *stepY,
        DigitalIoPin* lmXMin, DigitalIoPin* lmXMax, DigitalIoPin* lmYMin, DigitalIoPin* lmYMax):
    dirXPin(dirX), stepXPin(stepX), dirYPin(dirY), stepYPin(stepY),
    lmXMin(lmXMin), lmXMax(lmXMax), lmYMin(lmYMin), lmYMax(lmYMax) {

    dirToOrigin = false;
    sbRIT = xSemaphoreCreateBinary();
}

XYMotor::~XYMotor() {

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
    delta = 2 * stepY - stepX;
    motorYMove = (delta > 0) ? true : false;
    isUpdateDelta = false;

    RIT_start(pps);
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
            stepXPin->write(xState);
            x += (xState) ? 1 : 0;
            xState = !xState;
        }

        if (motorYMove && (y < stepY)) {
            stepYPin->write(yState);
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

void XYMotor::setBaseX(int base) {
    baseX = base;
}

void XYMotor::setBaseY(int base) {
    baseY = base;
}

void XYMotor::setTotalStepX(int total) {
    totalStepX = total;
}

void XYMotor::setTotalStepY(int total) {
    totalStepY = total;
}

int calibrateMotor(DigitalIoPin* step, DigitalIoPin* dir, DigitalIoPin* lmMin, DigitalIoPin* lmMax) {
    int xstep;
    bool dirToOrigin = false;

    /* move to origin */
    dir->write(dirToOrigin);
    bool stepValue = false;
    while (!lmMin->read() && !lmMax->read()) {
        step->write(stepValue);
        stepValue = !stepValue;
        vTaskDelay(1);
    }

    /* first calibration */
    dir->write(!dirToOrigin);
    stepValue = false;
    while (!lmMax->read()) {
        xstep++;
        step->write(stepValue);
        stepValue = !stepValue;
        vTaskDelay(1);
    }

    /* second calibration */
    dir->write(dirToOrigin);
    stepValue = false;
    while (!lmMin->read()) {
        xstep++;
        step->write(stepValue);
        stepValue = !stepValue;
        vTaskDelay(1);
    }

    return xstep / 4;
}
