/*
 * StepperMotor.h
 *
 *  Created on: Sep 18, 2017
 *      Author: tuanngu
 */

#ifndef STEPPERMOTOR_H_
#define STEPPERMOTOR_H_
#include "DigitalIopin.h"
#include "timers.c"

class StepperMotor {
private:
	DigitalIoPin xMotor;
	DigitalIoPin xDir;
	DigitalIoPin yMotor;
	DigitalIoPin yDir;


public:
    StepperMotor();
    virtual ~StepperMotor();
    void moveX(bool direction, int step);
    void moveY(bool direction, int step);
};

#endif /* STEPPERMOTOR_H_ */
