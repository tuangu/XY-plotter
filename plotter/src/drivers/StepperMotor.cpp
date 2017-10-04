#include <StepperMotor.h>

#include "chip.h"

StepperMotor::StepperMotor(LPC_SCT_T *timer_, int rpm_, short mPort, short mPin, short dPort, short dPin):
        motorPort(mPort), motorPin(mPin), dirPort(dPort), dirPin(dPin) {
    if ((timer_ == LPC_SCT2) ||(timer_ == LPC_SCT3))
        this->timer = timer_;

    if (rpm_ > 0)
        this->rpm = rpm_;
    else
        this->rpm = 60;

    /* Hardware setup */
    Chip_SCT_Init(timer);
    timer->CONFIG |= (1 << 0); // 32-bit timer
    timer->LIMIT |= (1 << 0); // use event 0 as a counter limit
}

StepperMotor::~StepperMotor() {
}

void StepperMotor::move(float newPos) {

}

void StepperMotor::Timer_start(int count, int us) {


}

int StepperMotor::getRpm() {
    return rpm;
}

void StepperMotor::setRpm(int rpm) {
    if (rmp > 0)
        this->rpm = rpm;
}

short StepperMotor::getMotorPort() {
    return motorPort;
}

short StepperMotor::getMotorPin() {
    return motorPin;
}

short StepperMotor::getDirPort() {
    return dirPort;
}

short StepperMotor::getDirPin() {
    return dirPin;
}
