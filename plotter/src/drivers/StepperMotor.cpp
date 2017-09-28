/*
 * StepperMotor.cpp
 *
 *  Created on: Sep 18, 2017
 *      Author: tuanngu
 */

#include <StepperMotor.h>

StepperMotor::StepperMotor() {
    // TODO Auto-generated constructor stub
	this->xMotor = DigitalIoPin(0, 27, DigitalIoPin::output, true);
	this->xDir = DigitalIoPin(0, 28, DigitalIoPin::output, true);
	this->yMotor = DigitalIoPin(0, 24, DigitalIoPin::output, true);
	this->yDir = DigitalIoPin(1,0, DigitalIoPin::output, true);
}

StepperMotor::~StepperMotor() {
    // TODO Auto-generated destructor stub
}

void StepperMotor::moveX(bool direction, int step){
	xDir.write(direction);
	for(int i = 0; i < step; i++){
		xMotor.write(true);
		vTaskDelay(1);
		xMotor.write(false);
		vTaskDelay(1);
	}
}

void StepperMotor::moveY(bool direction, int step){
	yDir.write(direction);
	for(int i = 0; i < step; i++){
		yMotor.write(true);
		vTaskDelay(1);
		yMotor.write(false);
		vTaskDelay(1);
	}
}
