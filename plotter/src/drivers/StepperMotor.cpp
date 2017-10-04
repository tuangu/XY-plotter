#include <StepperMotor.h>

#include "chip.h"
#include "DigitalIoPin.h"

#include "FreeRTOS.h"
#include "semphr.h"

StepperMotor::StepperMotor(LPC_SCT_T *timer_, int rpm_, short mPort, short mPin, short dPort, short dPin):
        timer(timer_) {

	/* Select timer, irq handler*/
    if (timer_ == LPC_SCT2) {
        this->irq = SCT2_IRQn;
        base = 380;
    } else if (timer_ == LPC_SCT3) {
        this->irq = SCT3_IRQn;
        base = 310;
    }

    /* initialize step and direction pin */
    stepPin = DigitalIoPin(mPort, mPin, DigitalIoPin::pullup, true);
    dirPin = DigitalIoPin(dPort, dPin, DigitalIoPin::pullup, true);

    /* Speed */
    this->rpm = (rpm_ > 0) ? rpm_ : 60;

    /* Binary semaphore */
    this->sbTimer = xSemaphoreCreateBinary();

    /* Hardware setup */
    Chip_SCT_Init(timer);
    timer->CONFIG |= (1 << 0) | (1 << 17);  // 32-bit timer, auto limit
    timer->CTRL_U |= (0 << 5);              // set prescaler = 1, % Timer freq = 72 Mhz
}

StepperMotor::~StepperMotor() {
}

void StepperMotor::move(float newPos) {
	float distance = abs(newPos - currentPosition);
	if(newPos - currentPosition < 0){
		diretion = !diretion;
	}
	totalStep = base/distance;
	Timer_start(totalStep);
}

void StepperMotor::Timer_start(int count) {
    uint64_t cmpValue = Chip_Clock_GetSystemClockRate() * 60 / (rpm * motorPulsePerRevolution);

    NVIC_DisableIRQ(irq);
    Timer_count = count;

    timer->COUNT_U 			= (1 << 3);		// clear the unified counter
    timer->MATCHREL[0].U    = cmpValue;		// set value of match 0

    timer->EVENT[0].STATE   = 0xFFFFFFFF;	// event 0 happens in all states
    timer->EVENT[0].CTRL    = (1 << 12);	// match 0 condition only

    timer->EVEN             = (1 << 0);     // event 0 generates an interrupt

    NVIC_EnableIRQ(irq);

    timer->CTRL_U           &= ~(1 << 2);   // unhalt by clearing bit 2

    // Wait for ISR
    if (xSemaphoreTake(sbTimer, portMAX_DELAY) == pdTRUE) {
        NVIC_DisableIRQ(irq);
    }
}

bool StepperMotor::irqHandler() {
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;

	if (Timer_count > 0) {
		Timer_count--;

		// TODO Handle limit switches

		stepPin.write(stepValue);
		stepValue = !stepValue;

#ifdef DEBUG
        if ((RIT_count % 100) == 0) {
            Board_LED_Toggle(2);
        }
#endif

	} else {
		timer->CTRL_U 		= (1 << 2);		// halt by setting bit 2
		xSemaphoreGiveFromISR(sbTimer, &xHigherPriorityWoken);
	}

	return xHigherPriorityWoken;
}

void StepperMotor::setTotalStep(int steps) {
	totalStep = steps;
}

int StepperMotor::getRpm() {
    return rpm;
}

void StepperMotor::setRpm(int rpm) {
    if (rmp > 0)
        this->rpm = rpm;
}

void StepperMotor::setCurrentPosition(float newPos) {
	currentPosition = newPos;
}
