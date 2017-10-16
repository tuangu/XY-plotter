#include <XYMotor.h>

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

    errXAxis = 0;
    errYAxis = 0;
    totalStepX = 0;
    totalStepY = 0;
    isCalibrating = true;
}

XYMotor::~XYMotor() {

}

void XYMotor::SetXStepInMM(int base){
	xSPMM = totalStepX / base;
}

void XYMotor::SetYStepInMM(int base){
	ySPMM = totalStepY / base;
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
        RIT_start(pps);

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

void XYMotor::move(float toX, float toY, int pps) {
    float dx = round(xSPMM*toX) - currentX;
    float dy = round(ySPMM*toY) - currentY;

    dirX = (dx < 0) ? dirXToOrigin : !dirXToOrigin;
    dirXPin->write(dirX);
    dirY = (dy < 0) ? dirYToOrigin : !dirYToOrigin;
    dirYPin->write(dirY);

    x = 0; xState = false;
    y = 0; yState = false;

    float tempStepX = fabs(xSPMM*toX - currentX);
    float tempStepY = fabs(ySPMM*toY - currentY);

    if (tempStepX < tempStepY) { // swap X and Y axis for better resolution
        tempStepX = fabs(ySPMM*toY - currentY);
        tempStepY = fabs(xSPMM*toX - currentX);

        stepX = fabs(dy);
        stepY = fabs(dx);

        tempXPin = stepYPin;
        tempYPin = stepXPin;
    } else {
    	stepX = fabs(dx);   // round to the nearest even
    	stepY = fabs(dy);

        tempXPin = stepXPin;
        tempYPin = stepYPin;
    }

    delta = 2 * stepY - stepX;
    motorYMove = (delta > 0) ? true : false;

    isUpdateDelta = false;

	char buffer[48];
	snprintf(buffer, 48, "X: %d,Y: %d, cX: %.2f, cY: %.2f\r\n", stepX, stepY, currentX, currentY);
	ITM_write(buffer);

    RIT_start(pps);
    currentX = round(xSPMM*toX);
    currentY = round(ySPMM*toY);
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
        bool minX = lmXMin->read() && (dirX == dirXToOrigin);
        bool maxX = lmXMax->read() && (dirX == !dirXToOrigin);
        bool minY = lmYMin->read() && (dirY == dirYToOrigin);
        bool maxY = lmYMax->read() && (dirY == !dirYToOrigin);

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

int XYMotor::getTotalStepX() {
	return totalStepX;
}

int XYMotor::getTotalStepY() {
	return totalStepY;
}
