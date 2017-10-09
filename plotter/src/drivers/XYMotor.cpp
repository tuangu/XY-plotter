#include <XYMotor.h>

#include <stdlib.h>
#include "DigitalIoPin.h"
#include "FreeRTOS.h"
#include "semphr.h"

XYMotor::XYMotor(DigitalIoPin* dirX, DigitalIoPin* stepX, DigitalIoPin* dirY, DigitalIoPin *stepY):
    dirXPin(dirX), stepXPin(stepX), dirYPin(dirY), stepYPin(stepY) {
    sbRIT = xSemaphoreCreateBinary();
}

XYMotor::~XYMotor() {

}

void XYMotor::move(float fromX, float fromY, float toX, float toY) {
    float dx = toX - fromX;
    float dy = toY - fromY;

    if (dx < 0)
        dirXPin->write(dirToOrigin);
    else
        dirXPin->write(!dirToOrigin);

    if (dy < 0)
        dirYPin->write(dirToOrigin);
    else
        dirYPin->write(!dirToOrigin);

    x = 0; xState = false;
    y = 0; yState = false;
    stepX = abs(dx * totalStepX / baseX);
    stepY = abs(dy * totalStepY / baseY);
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
        // move motor X, update x every 2 interrupts
        if (x < stepX) {
            stepXPin->write(xState);
            x += (xState) ? 1 : 0;
            xState = !xState;
        }

        // move motor Y
        // need 2 interrupts to drive step pin from LOW to HIGH
        if (motorYMove == true) {
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

void XYMotor::setPps(int newPps) {
    pps = newPps;
}

int XYMotor::getPps() {
    return pps;
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
